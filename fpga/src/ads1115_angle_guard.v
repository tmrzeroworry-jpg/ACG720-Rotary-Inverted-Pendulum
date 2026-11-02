// Signed ADS1115 conversion word -> canonical angle code for this single-ended
// potentiometer. Raw ADC remains unmodified in UART. No assumption that every
// negative word is harmless offset: only -64..-1 (<=8mV at PGA4.096V) clamps.
// The positive endpoint tolerance512 (64mV) covers recorded max26868. Its
// overflow is circular, preserving original positive-word angular differences.
module ads1115_angle_guard #(
    parameter integer FULL_SCALE=26400,
    parameter integer NEGATIVE_TOLERANCE=64,
    parameter integer POSITIVE_MARGIN=512
)(
    input wire [15:0] adc_word,
    input wire adc_valid,
    output wire word_legal,
    output wire sample_valid,
    output wire sample_rejected,
    output wire sample_clamped,
    output wire sample_wrapped,
    output wire [15:0] angle_code
);
    wire signed [16:0] signed_code=$signed({adc_word[15],adc_word});
    wire near_zero_negative=signed_code<0 && signed_code>=-NEGATIVE_TOLERANCE;
    wire positive_ok=signed_code>=0 && signed_code<=FULL_SCALE+POSITIVE_MARGIN;
    assign word_legal=near_zero_negative || positive_ok;
    assign sample_valid=adc_valid && word_legal;
    assign sample_rejected=adc_valid && !word_legal;
    assign sample_clamped=adc_valid && near_zero_negative;
    assign sample_wrapped=adc_valid && positive_ok && signed_code>=FULL_SCALE;
    assign angle_code=near_zero_negative ? 16'd0 :
        positive_ok ? (signed_code>=FULL_SCALE ? adc_word-FULL_SCALE : adc_word) : 16'd0;
endmodule
