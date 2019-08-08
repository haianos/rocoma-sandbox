/*
 * Enea Scioni
 * 09/08/2019
 * 
 * This State definition is importet from `rocoma_test`. 
 * In fact, the one suggested in rocoma_example/rocomaex_model is "broken" -> no "setValue" method.
 */

/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Roco state implementation for testing.
 */

#pragma once

#include <message_logger/message_logger.hpp>
#include <roco/model/StateInterface.hpp>

namespace rocomaex_model { //NOTE: namespace has been changed

class RocoState : public roco::StateInterface {
 public:
  bool checkState() const override {
    if (value_ > maxValue_) {
      MELO_ERROR("[rocoma_test::RocoState]: Checking State Error. Value %f bigger than maximum allowed %f", value_, maxValue_);
      return false;
    }
    return true;
  }

  void setValue(double value) { value_ = value; }
  double getValue() const { return value_; }

  static constexpr double maxValue_ = 5.0;

 private:
  double value_;
};

}  // namespace rocoma_test