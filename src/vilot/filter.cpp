#include "vilot/filter.h"
#include <cmath>

namespace vilot {

constexpr float s_curve_filter(float input, float strength,
                               float max_input) noexcept {
  float abs_max_input = std::abs(max_input);
  float weight = std::exp(-0.1 * strength);
  return (weight +
          exp(0.1 * (std::abs(input) - abs_max_input)) * (1.0 - weight)) *
         input;
}

constexpr float ramp_curve_filter(float input, float strength,
                                  float max_input) noexcept {
  float abs_max_input = std::abs(max_input);
  float weight = std::exp(-0.1 * strength);
  return (weight +
          exp(0.1 * (std::abs(input) - abs_max_input)) * (1.0 - weight)) *
         input;
}

constexpr float deadzone_filter(float input, float min_output,
                                float max_output) noexcept {
  float abs_min_output = std::abs(min_output);
  float abs_max_output = std::abs(max_output);
  if (input < abs_min_output && input > -abs_min_output) {
    return 0.0;
  }
  return std::copysign((input - std::copysign(abs_min_output, input)) /
                           (1 - abs_min_output / abs_max_output),
                       input);
}

BiquadLowPassFilter::BiquadLowPassFilter(float sample_rate, float cutoff_freq) {
  init(sample_rate, cutoff_freq);
  z1 = z2 = 0.0;
}

void BiquadLowPassFilter::reset() { z1 = z2 = 0.0; }

void BiquadLowPassFilter::set_cutoff_frequency(float sample_rate,
                                               float cutoff_freq) {
  init(sample_rate, cutoff_freq);
}

float BiquadLowPassFilter::apply(float input) {
  float output = a0 * input + a1 * z1 + a2 * z2 - b1 * z1 - b2 * z2;
  z2 = z1;
  z1 = input;
  return output;
}

void BiquadLowPassFilter::init(float sample_rate, float cutoff_freq) {
  float omega = 2.0 * M_PI * cutoff_freq / sample_rate;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0 * 0.707);

  a0 = (1.0 - cos_omega) / 2.0;
  a1 = 1.0 - cos_omega;
  a2 = a0;
  b1 = -2.0 * cos_omega;
  b2 = 1.0 - alpha;

  float a0_inv = 1.0 / (1.0 + alpha);
  a0 *= a0_inv;
  a1 *= a0_inv;
  a2 *= a0_inv;
  b1 *= a0_inv;
  b2 *= a0_inv;
}

constexpr float exp_decay_filter(const float a, const float b,
                                 const float decay,
                                 const units::time::second_t dt) noexcept {
  return b + (a - b) * std::exp(-decay * dt.value());
}

ExpDecayFilter::ExpDecayFilter(float decay) : decay(decay) {}

float ExpDecayFilter::apply(float input, units::time::second_t dt) {
  float val = input + (prev - input) * std::exp(-decay * dt.value());
  this->prev = val;
  return val;
}

void ExpDecayFilter::reset() { this->prev = 0; }

float input_modulus_filter(float input, float min_input, float max_input) {
  float range = max_input - min_input;

  // Normalize input to [0, range)
  float normalized = std::fmod(input - min_input, range);

  // Handle negative results from fmod
  if (normalized < 0) {
    normalized += range;
  }

  // Shift back to [min_input, max_input)
  return normalized + min_input;
}

InputModulusFilter::InputModulusFilter(float min_input, float max_input)
    : min_input(min_input), max_input(max_input) {}

float InputModulusFilter::apply(float input) {
  return input_modulus_filter(input, this->min_input, this->max_input);
}

float InputModulusFilter::get_min_input() { return this->min_input; }

float InputModulusFilter::get_max_input() { return this->max_input; }

} // namespace vilot
