def graceful_stop(arm_sequence):
    """Simple helper to request a graceful stop on an ArmSequence instance."""
    if arm_sequence is None:
        return
    try:
        arm_sequence.Stop()
    except Exception:
        print("Warning: failed to request stop on arm_sequence")
