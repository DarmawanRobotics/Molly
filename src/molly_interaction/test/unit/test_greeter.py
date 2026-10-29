from __future__ import annotations

from molly_interaction.core.greeter import (
    Greeter,
    GreeterConfig,
    GreeterState,
    load_choice,
    next_text,
    save_choice,
)


def greeter(**kwargs):
    """Return a greeter with short times for the tests."""
    return Greeter(
        GreeterConfig(
            **{'approach_distance': 1.5, 'dwell_sec': 1.0, 'cooldown_sec': 10.0, **kwargs}
        )
    )


def test_greets_after_the_visitor_stays():
    """Somebody close in front for dwell_sec is greeted once."""
    g = greeter()
    assert not g.update(0.0, 1.0, False) and g.state is GreeterState.WATCHING
    assert not g.update(0.5, 1.2, False)
    assert g.update(1.1, 1.2, False) and g.state is GreeterState.GREETING
    assert g.last_greet_age(3.1) == 2.0


def test_far_or_nobody_does_not_count():
    """Visitors beyond approach_distance or nobody (-1 / None) never trigger a greeting."""
    g = greeter()
    for t in range(5):
        assert not g.update(float(t), 3.0, False)
        assert not g.update(float(t) + 0.5, -1.0, False)
        assert not g.update(float(t) + 0.7, None, False)
    assert g.state is GreeterState.WAITING and g.last_greet_age(5.0) is None


def test_same_visitor_not_greeted_twice():
    """After a greeting the front must empty and the cooldown pass before the next one."""
    g = greeter()
    g.update(0.0, 1.0, False)
    assert g.update(1.0, 1.0, False)
    assert not any(g.update(t, 1.0, False) for t in (2.0, 15.0, 30.0))  # still the same one
    assert g.state is GreeterState.GREETED
    g.update(31.0, -1.0, False)  # they leave
    g.update(32.0, 1.0, False)  # somebody new
    assert g.update(33.0, 1.0, False)


def test_cooldown_after_a_greeting():
    """A new visitor right after a greeting waits for the cooldown."""
    g = greeter()
    g.update(0.0, 1.0, False)
    assert g.update(1.0, 1.0, False)
    g.update(2.0, None, False)
    g.update(3.0, 1.0, False)
    assert not g.update(5.0, 1.0, False) and g.state is GreeterState.COOLDOWN
    assert g.update(11.5, 1.0, False)


def test_busy_and_disabled_pause_the_greeter():
    """No greeting while busy (tour, walking, talking) or switched off; the dwell restarts."""
    g = greeter()
    g.update(0.0, 1.0, False)
    assert not g.update(2.0, 1.0, True) and g.state is GreeterState.PAUSED
    assert not g.update(2.5, 1.0, False)  # dwell starts again after busy
    assert g.update(3.6, 1.0, False)
    g = greeter()
    g.enabled = False
    assert not g.update(0.0, 1.0, False) and not g.update(5.0, 1.0, False)
    assert g.state is GreeterState.OFF


def test_manual_greeting_counts_as_greeted():
    """greeted() (interaction/greet) also keeps the same visitor from a second greeting."""
    g = greeter()
    g.update(0.0, 1.0, False)
    g.greeted(0.5)
    assert not g.update(2.0, 1.0, False) and g.state is GreeterState.GREETED


def test_next_text_goes_round():
    """Texts are used round robin; an empty list gives nothing."""
    texts = ['a', 'b']
    text, i = next_text(texts, 0)
    assert (text, i) == ('a', 1)
    assert next_text(texts, i) == ('b', 0)
    assert next_text([], 3) == ('', 0)


def test_choice_is_kept(tmp_path):
    path = str(tmp_path / 'greeter.json')
    assert load_choice(path) == (None, '')
    save_choice(path, False)
    assert load_choice(path) == (False, '')
    (tmp_path / 'broken.json').write_text('{oops')
    value, problem = load_choice(str(tmp_path / 'broken.json'))
    assert value is None and problem
    assert load_choice('') == (None, '')
