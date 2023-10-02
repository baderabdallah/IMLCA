
#ifndef FINITE_STATE_MACHINE_H
#define FINITE_STATE_MACHINE_H

#include <algorithm>
#include <functional>
#include <vector>

template <typename State, typename Event>
class FiniteStateMachine
{
  public:
    using Action =  std::function<void()>;

    explicit FiniteStateMachine(const State initial_state);

    void AddTransition(const State initial_state, const Event event, const State new_state, const Action action);
    void HandleEvent(const Event event);
    State GetCurrentState() const;

  private:
    struct Transition
    {
        State initial_state{};
        Event event{};
        State new_state{};
        Action action{nullptr};
    };

    State current_state_{};
    std::vector<Transition> transitions_{};
};

template <typename State, typename Event>
FiniteStateMachine<State, Event>::FiniteStateMachine(const State initial_state)
    : current_state_{initial_state}
{
}

template <typename State, typename Event>
void FiniteStateMachine<State, Event>::AddTransition(const State initial_state,
                                                     const Event event,
                                                     const State new_state,
                                                     const Action action)
{
    Transition transition{initial_state, event, new_state, action};
    transitions_.push_back(transition);
}

template <typename State, typename Event>
void FiniteStateMachine<State, Event>::HandleEvent(const Event event)
{
    auto transition_is_relevant = [this, event](const Transition& transition) {
        return ((transition.initial_state == this->current_state_) && (transition.event == event));
    };

    const auto it{std::find_if(std::begin(transitions_), std::end(transitions_), transition_is_relevant)};
    const auto transition_found{it != std::end(transitions_)};

    if (transition_found)
    {
        current_state_ = it->new_state;

        if (it->action)
        {
            it->action();
        }
    }
}

template <typename State, typename Event>
State FiniteStateMachine<State, Event>::GetCurrentState() const
{
    return current_state_;
}

#endif // FINITE_STATE_MACHINE_H
