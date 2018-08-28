#include "drake/tools/vector_gen/test/gen/sample_translator.h"

// GENERATED FILE DO NOT EDIT
// See drake/tools/lcm_vector_gen.py.

#include <stdexcept>

#include "drake/common/drake_assert.h"

namespace drake {
namespace tools {
namespace test {

<<<<<<< HEAD
std::unique_ptr<drake::systems::BasicVector<double>>
=======
std::unique_ptr<systems::BasicVector<double>>
>>>>>>> intial
SampleTranslator::AllocateOutputVector() const {
  return std::make_unique<Sample<double>>();
}

void SampleTranslator::Serialize(
<<<<<<< HEAD
    double time, const drake::systems::VectorBase<double>& vector_base,
=======
    double time, const systems::VectorBase<double>& vector_base,
>>>>>>> intial
    std::vector<uint8_t>* lcm_message_bytes) const {
  const auto* const vector = dynamic_cast<const Sample<double>*>(&vector_base);
  DRAKE_DEMAND(vector != nullptr);
  drake::lcmt_sample_t message;
  message.timestamp = static_cast<int64_t>(time * 1000);
  message.x = vector->x();
  message.two_word = vector->two_word();
  message.absone = vector->absone();
<<<<<<< HEAD
  message.unset = vector->unset();
=======
>>>>>>> intial
  const int lcm_message_length = message.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  message.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

void SampleTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
<<<<<<< HEAD
    drake::systems::VectorBase<double>* vector_base) const {
=======
    systems::VectorBase<double>* vector_base) const {
>>>>>>> intial
  DRAKE_DEMAND(vector_base != nullptr);
  auto* const my_vector = dynamic_cast<Sample<double>*>(vector_base);
  DRAKE_DEMAND(my_vector != nullptr);

  drake::lcmt_sample_t message;
  int status = message.decode(lcm_message_bytes, 0, lcm_message_length);
  if (status < 0) {
    throw std::runtime_error("Failed to decode LCM message sample.");
  }
  my_vector->set_x(message.x);
  my_vector->set_two_word(message.two_word);
  my_vector->set_absone(message.absone);
<<<<<<< HEAD
  my_vector->set_unset(message.unset);
=======
>>>>>>> intial
}

}  // namespace test
}  // namespace tools
}  // namespace drake
