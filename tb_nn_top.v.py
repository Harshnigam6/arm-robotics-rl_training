`timescale 1ns/1ps

module tb_nn_top;

  //----------------------------------
  // 1) Parameters & Local Variables
  //----------------------------------
  localparam NUM_TESTS = 5;

  // Clock (if your design needs one)
  reg clk;

  // DIP switch inputs (10 bits)
  reg [9:0] tb_dip_switches;

  // 7-segment output from DUT (assuming active-low segments, 7 bits)
  wire [6:0] tb_seg_out;

  // Test vectors: DIP inputs + expected 7-seg pattern
  reg [9:0] test_input [0:NUM_TESTS-1];
  reg [6:0] expected_seg[0:NUM_TESTS-1];

  integer i;
  integer errors = 0;


  //----------------------------------
  // 2) Instantiate the DUT
  //----------------------------------
  // NOTE: Replace "nn_top" with your actual top module name.
  //       Match port names with your design.
  nn_top DUT (
    .clk          (clk),                // If your design uses a clock
    .dip_switches (tb_dip_switches),
    .seg_out      (tb_seg_out)
    // ... add other signals if needed ...
  );


  //----------------------------------
  // 3) Clock Generation (if needed)
  //----------------------------------
  initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns period => 100 MHz
  end


  //----------------------------------
  // 4) Initialize Test Vectors
  //----------------------------------
  initial begin
    // NOTE: These are dummy values, just as an example.

    // DIP [9:0] -> Expected 7-seg pattern (tb_seg_out).
    // You can fill in real expected patterns from your offline calculations.
    test_input[0]    = 10'b0000000000;  expected_seg[0] = 7'b1000000; // e.g. "0"
    test_input[1]    = 10'b0000000001;  expected_seg[1] = 7'b1111001; // e.g. "1"
    test_input[2]    = 10'b0000000010;  expected_seg[2] = 7'b0100100; // e.g. "2"
    test_input[3]    = 10'b0000000101;  expected_seg[3] = 7'b0000110; // e.g. "9"? (dummy)
    test_input[4]    = 10'b1111111111;  expected_seg[4] = 7'b0000000; // All-on or all-off? (dummy)

    // If your 7-seg is active-low, you might invert these bits. Adjust as needed!
  end


  //----------------------------------
  // 5) Main Test Procedure
  //----------------------------------
  initial begin
    // Wait a few cycles for reset (if your DUT has an internal reset)
    #20;

    // Go through each test pattern
    for (i = 0; i < NUM_TESTS; i = i + 1) begin
      // Apply DIP switch input
      tb_dip_switches = test_input[i];

      // Wait some cycles for the DUT to update, 
      // especially if there's a pipeline or sequential logic
      #20;  // adjust as needed

      // Check the 7-segment output
      if (tb_seg_out !== expected_seg[i]) begin
        $display("TEST %0d FAILED:", i);
        $display("  Input DIP  = %b", test_input[i]);
        $display("  Expected   = %b", expected_seg[i]);
        $display("  Got        = %b", tb_seg_out);
        errors = errors + 1;
      end else begin
        $display("TEST %0d PASSED: Input = %b, Output = %b",
                  i, test_input[i], tb_seg_out);
      end

      // Extra delay before next test
      #10;
    end

    // Final Summary
    if (errors == 0) begin
      $display("All %0d tests PASSED!", NUM_TESTS);
    end else begin
      $display("%0d tests FAILED.", errors);
    end

    $stop;  // End simulation
  end

endmodule
