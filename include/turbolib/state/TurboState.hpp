#pragma once

#include <functional>

template <typename StateType>
class TurboState {
 public:
  using ApplyFunc = std::function<void(const StateType&)>;
  using HookFunc = std::function<void(const StateType&)>;

  explicit TurboState(StateType initialState, ApplyFunc applyFunc)
      : currentState(std::move(initialState)), applyFunc(std::move(applyFunc)) {}

  void Set(const StateType& newState) {
    if (onExit) {
      onExit(currentState);
    }

    currentState = newState;

    if (onEnter) {
      onEnter(newState);
    }
  }

  const StateType& Get() const { return currentState; }

  void Apply() const { applyFunc(currentState); }

 private:
  ApplyFunc applyFunc;
  StateType currentState;
  HookFunc onEnter{};
  HookFunc onExit{};
};
