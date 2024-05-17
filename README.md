Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.

This provided SensorAPI is a preliminary release for the Product `BMA580`.

This SensorAPI implementation refers to the preliminary Datasheet revision `0.7`.

Since the datasheet and this release of the SensorAPI is still preliminary, there are few deviations between the SensorAPI and the datasheet:
* In case of any discrepancy, please refer to the datasheet.
* In general, small deviations in naming might be present.
* Extended MemoryMap: The naming of some of the registers and the inclusive fields may vary inside the Extended MemoryMap. Despite the mismatch in naming or description, the functionality is not affected. 
* FIFO-Examples: In some corner cases (especially with the “stop-on-full” mode) the given examples in SensorAPI might not provide the optimal performance.
* As this is preliminary release, there might be some registers in the datasheet that are currently not supported in the SensorAPI and will be covered in the future release.

For more details, please contact your Bosch Sensortec representative.
