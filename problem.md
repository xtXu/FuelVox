# Problem List

1. `catkin build FuelVox`error: 

   ```bash
   fatal error: : multi_map_server/MultiOccupancyGrid.h such file or directory
   ```

   It seems that the `MultiOccupancyGrid.h` is auto generated by custom `msg` file. However, `catkin build`builds the packages isolated and the `message_runtime`doesn't run to generate the corresponding headers.

   **Fix:** `catkin build multi_map_server` first, then `catkin build`
