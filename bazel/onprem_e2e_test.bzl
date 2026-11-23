"""OMTS wrapper around intrinsic_onprem_e2e_test() that pins the inctl binary."""

load("@intrinsic_onprem_e2e_test//:defs.bzl", _intrinsic_onprem_e2e_test = "intrinsic_onprem_e2e_test")

def onprem_e2e_test(name, **kwargs):
    """Runs `test` against a VM leased from a VM pool.

    Args:
      name: Name of the test target.
      **kwargs: Forwarded to intrinsic_onprem_e2e_test().
    """
    _intrinsic_onprem_e2e_test(
        name = name,
        inctl = Label("@intrinsic-core//intrinsic_sdk/intrinsic/tools/inctl:inctl_external"),
        **kwargs
    )
