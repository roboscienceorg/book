Localization Belief
-------------------

This section under development.

The process of localization is stating a belief that the robot is found
at a given location. There is ambiguity in our knowledge which implies
this belief can be represented as a probability distribution. We can
arbitrarily decide that this belief, the probability distribution, is a
normal distribution or something more exotic. Having a single “bump”
means we have one hypothesis of location, a *single hypothesis*. Having
a distribution with multiple bumps means we have *multiple hypotheses*
of the robot location.

|Single and Multiple Hypotheses.|   |Single and Multiple Hypotheses.|

Having multiple hypotheses seems a bit odd at first, but actually
arises. Imagine you have a Starbucks map - a map of a city that just
shows Starbucks. Also assume you drive up to a Starbucks. Now compare to
the map. You can now isolate your position to one of the :math:`n`
Starbucks locations on the map. This is an example of multiple
hypotheses. Only until you receive additional information are you able
to break the ambiguity. With the reduced information available to a
robot, this situation arises when faced with vision system that use
corners and walls to generate landmarks.
