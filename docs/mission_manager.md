# Mission manager

M01–M04 form one contiguous loop:

`S1_STORAGE → S2_ASSEMBLY → S3_INSPECTION → S4_PACKAGING → S1_STORAGE`.

The manager sends `NavigateToPose` goals, waits for loading, owns one cargo ID,
approaches the destination staging pose, calls `DockRobot`, validates the final
pose, unloads and publishes `COMPLETED`. State and cargo are observable on
`/mission/state` and `/mission/cargo`.

Failures return to the last retryable state instead of silently advancing.
There can never be more than one loaded cargo item.
