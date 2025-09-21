// arys-fault-fsm.sv
// SystemVerilog implementation of Fault Detection FSM for Arys Garage Q2
// States: NORMAL -> WARNING -> FAULT -> SHUTDOWN
// Features: debounce, persistence, fault priority, masking
// Includes: DUT (fault_fsm), parameterized counters, testbench (tb_fault_fsm)
// Run with: iverilog -g2012 arys-fault-fsm.sv -o simv && vvp simv

`timescale 1ns/1ps

module fault_fsm #(
    parameter DEBOUNCE_BITS = 6,       // debounce counter width (samples)
    parameter PERSIST_BITS  = 12,      // persistence counter width (samples)
    parameter NUM_CELLS     = 4        // number of cell voltage flags
)(
    input  logic clk,
    input  logic rst_n,

    // Raw fault flags (combinational inputs from sensors/monitors)
    input  logic [NUM_CELLS-1:0] cell_overvoltage, // per-cell OV flag
    input  logic [NUM_CELLS-1:0] cell_undervoltage,// per-cell UV flag
    input  logic current_overlimit,                 // current fault
    input  logic temp_overlimit,                    // temperature fault

    // Fault mask (1 = masked/ignored)
    input  logic [NUM_CELLS-1:0] mask_cell_ov,
    input  logic [NUM_CELLS-1:0] mask_cell_uv,
    input  logic mask_current,
    input  logic mask_temp,

    // Outputs
    output logic [1:0] state, // 00 NORMAL, 01 WARNING, 10 FAULT, 11 SHUTDOWN
    output logic warning,     // warning asserted in WARNING state
    output logic fault_latched,// latched fault (stays until reset)
    output logic shutdown     // shutdown asserted in SHUTDOWN
);

    typedef enum logic [1:0] {S_NORMAL = 2'b00, S_WARNING = 2'b01, S_FAULT = 2'b10, S_SHUT = 2'b11} state_t;
    state_t cur_state, nxt_state;

    // Effective (masked) flags
    logic [NUM_CELLS-1:0] cell_ov_eff, cell_uv_eff;
    logic current_eff, temp_eff;

    assign cell_ov_eff = cell_overvoltage & ~mask_cell_ov;
    assign cell_uv_eff = cell_undervoltage & ~mask_cell_uv;
    assign current_eff = current_overlimit & ~mask_current;
    assign temp_eff    = temp_overlimit & ~mask_temp;

    // Priority encoding: assign a fault code priority order
    // Highest priority: temperature, then current, then cell OV, then cell UV
    logic temp_fault_req, current_fault_req, cellov_fault_req, celluv_fault_req;
    assign temp_fault_req    = |temp_eff;
    assign current_fault_req = current_eff;
    assign cellov_fault_req  = |cell_ov_eff;
    assign celluv_fault_req  = |cell_uv_eff;

    // Debounce / persistence counters for each fault type
    logic [DEBOUNCE_BITS-1:0] db_temp, db_current;
    logic [DEBOUNCE_BITS-1:0] db_cellov [NUM_CELLS-1:0];
    logic [DEBOUNCE_BITS-1:0] db_celluv [NUM_CELLS-1:0];

    // Persistence counters (counts consecutive samples of active condition)
    logic [PERSIST_BITS-1:0] pst_temp, pst_current;
    logic [PERSIST_BITS-1:0] pst_cellov [NUM_CELLS-1:0];
    logic [PERSIST_BITS-1:0] pst_celluv [NUM_CELLS-1:0];

    // Thresholds (configurable by parameters or localparams)
    localparam logic [DEBOUNCE_BITS-1:0] DB_THRESH = {DEBOUNCE_BITS{1'b1}}; // max -> simple: require full counter
    localparam logic [PERSIST_BITS-1:0] PST_THRESH = 12'd100; // e.g., 100 cycles of persistence

    // Internal signals: event asserted when debounced+persistent
    logic temp_event, current_event;
    logic [NUM_CELLS-1:0] cellov_event, celluv_event;

    // Latch for fault (sticky) until reset
    logic fault_sticky;

    // -----------------------
    // Debounce & Persistence logic
    // -----------------------
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_temp <= 0; db_current <= 0; pst_temp <= 0; pst_current <= 0;
            for (i=0;i<NUM_CELLS;i=i+1) begin
                db_cellov[i] <= 0; db_celluv[i] <= 0;
                pst_cellov[i] <= 0; pst_celluv[i] <= 0;
            end
        end else begin
            // Temperature
            if (temp_fault_req) begin
                if (db_temp != DB_THRESH) db_temp <= db_temp + 1;
            end else db_temp <= 0;
            if (db_temp == DB_THRESH) begin
                if (pst_temp != PST_THRESH) pst_temp <= pst_temp + 1;
            end else pst_temp <= 0;

            // Current
            if (current_fault_req) begin
                if (db_current != DB_THRESH) db_current <= db_current + 1;
            end else db_current <= 0;
            if (db_current == DB_THRESH) begin
                if (pst_current != PST_THRESH) pst_current <= pst_current + 1;
            end else pst_current <= 0;

            // Cells
            for (i=0;i<NUM_CELLS;i=i+1) begin
                // OV
                if (cell_ov_eff[i]) begin
                    if (db_cellov[i] != DB_THRESH) db_cellov[i] <= db_cellov[i] + 1;
                end else db_cellov[i] <= 0;
                if (db_cellov[i] == DB_THRESH) begin
                    if (pst_cellov[i] != PST_THRESH) pst_cellov[i] <= pst_cellov[i] + 1;
                end else pst_cellov[i] <= 0;

                // UV
                if (cell_uv_eff[i]) begin
                    if (db_celluv[i] != DB_THRESH) db_celluv[i] <= db_celluv[i] + 1;
                end else db_celluv[i] <= 0;
                if (db_celluv[i] == DB_THRESH) begin
                    if (pst_celluv[i] != PST_THRESH) pst_celluv[i] <= pst_celluv[i] + 1;
                end else pst_celluv[i] <= 0;
            end
        end
    end

    // Event when persistence reached
    assign temp_event    = (pst_temp == PST_THRESH);
    assign current_event = (pst_current == PST_THRESH);
    generate
        genvar gi;
        for (gi=0; gi<NUM_CELLS; gi=gi+1) begin : GEN_EVENTS
            assign cellov_event[gi] = (pst_cellov[gi] == PST_THRESH);
            assign celluv_event[gi] = (pst_celluv[gi] == PST_THRESH);
        end
    endgenerate

    // -----------------------
    // Fault priority resolver
    // -----------------------
    // Produce a single fault request with encoded type/prio
    typedef enum logic [2:0] {F_NONE=3'd0, F_TEMP=3'd1, F_CURRENT=3'd2, F_CELL_OV=3'd3, F_CELL_UV=3'd4} fault_t;
    fault_t req_fault;

    always_comb begin
        req_fault = F_NONE;
        if (temp_event) req_fault = F_TEMP;
        else if (current_event) req_fault = F_CURRENT;
        else begin
            // check per-cell OV first (find highest-indexed cell for example)
            req_fault = F_NONE;
            for (int j=NUM_CELLS-1; j>=0; j--) begin
                if (cellov_event[j]) begin req_fault = F_CELL_OV; disable for; end
            end
            if (req_fault == F_NONE) begin
                for (int j=NUM_CELLS-1; j>=0; j--) begin
                    if (celluv_event[j]) begin req_fault = F_CELL_UV; disable for; end
                end
            end
        end
    end

    // -----------------------
    // FSM: state transitions
    // -----------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_state <= S_NORMAL;
            fault_sticky <= 1'b0;
        end else begin
            cur_state <= nxt_state;
            // latch fault when entering FAULT
            if (nxt_state == S_FAULT) fault_sticky <= 1'b1;
        end
    end

    always_comb begin
        nxt_state = cur_state;
        case (cur_state)
            S_NORMAL: begin
                if (req_fault != F_NONE) nxt_state = S_WARNING;
            end
            S_WARNING: begin
                // If condition persists (sticky events still present) go to FAULT
                if (req_fault != F_NONE) begin
                    // small hysteresis: require req_fault same for certain cycles -> handled by persistence
                    nxt_state = S_FAULT;
                end else nxt_state = S_NORMAL;
            end
            S_FAULT: begin
                // escalate to shutdown on severe fault or if fault persists
                if (req_fault == F_TEMP) nxt_state = S_SHUT;
                else if (req_fault == F_CURRENT && fault_sticky) nxt_state = S_SHUT; // example logic
                else if (req_fault == F_NONE) nxt_state = S_NORMAL; // auto-recover if cleared
            end
            S_SHUT: begin
                // Only reset via external reset
                nxt_state = S_SHUT;
            end
            default: nxt_state = S_NORMAL;
        endcase
    end

    // Output assignments
    assign state = cur_state;
    assign warning = (cur_state == S_WARNING);
    assign fault_latched = fault_sticky;
    assign shutdown = (cur_state == S_SHUT);

endmodule

// -------------------------------
// Testbench: tb_fault_fsm
// -------------------------------
module tb_fault_fsm;
    parameter NUM_CELLS = 4;
    logic clk; logic rst_n;
    logic [NUM_CELLS-1:0] cell_ov, cell_uv;
    logic current_ol, temp_ol;
    logic [NUM_CELLS-1:0] mask_cell_ov, mask_cell_uv;
    logic mask_current, mask_temp;

    wire [1:0] state;
    wire warning, fault_latched, shutdown;

    fault_fsm #(.NUM_CELLS(NUM_CELLS)) dut (
        .clk(clk), .rst_n(rst_n),
        .cell_overvoltage(cell_ov), .cell_undervoltage(cell_uv),
        .current_overlimit(current_ol), .temp_overlimit(temp_ol),
        .mask_cell_ov(mask_cell_ov), .mask_cell_uv(mask_cell_uv),
        .mask_current(mask_current), .mask_temp(mask_temp),
        .state(state), .warning(warning), .fault_latched(fault_latched), .shutdown(shutdown)
    );

    // Clock
    initial clk = 0; always #5 clk = ~clk; // 100MHz -> period 10ns

    initial begin
        // VCD
        $dumpfile("tb_fault_fsm.vcd");
        $dumpvars(0,tb_fault_fsm);

        // init
        rst_n = 0;
        cell_ov = 0; cell_uv = 0; current_ol = 0; temp_ol = 0;
        mask_cell_ov = 0; mask_cell_uv = 0; mask_current = 0; mask_temp = 0;
        #20; rst_n = 1;

        // Normal operation for 200 cycles
        repeat(200) @(posedge clk);

        // Transient spike on cell 2 (shorter than persistence) -> should not go to FAULT
        cell_ov[2] = 1; repeat(5) @(posedge clk); cell_ov[2] = 0; // short spike
        repeat(100) @(posedge clk);

        // Persistent overcurrent -> should escalate to WARNING then FAULT
        current_ol = 1; repeat(500) @(posedge clk); // long enough
        current_ol = 0; repeat(200) @(posedge clk);

        // Mask current and cause cell OV persistent -> should handle masked current and pick cell OV
        mask_current = 1; cell_ov[1] = 1; repeat(600) @(posedge clk);

        // Temperature severe event -> immediate priority -> shutdown
        temp_ol = 1; repeat(300) @(posedge clk);

        // End simulation
        $display("Simulation complete");
        #50 $finish;
    end
endmodule
