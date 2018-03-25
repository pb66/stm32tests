This project follows the first 3 parts of a tutorial by dBC in the ["STM Development"](https://community.openenergymonitor.org/t/stm32-development/6815?u=pb66
) thread on the [OpenEnergyMonitor forum](https://community.openenergymonitor.org/latest?u=pb66).

- [ADC Tutorial Part1, the GUI.](https://community.openenergymonitor.org/t/stm32-development/6815/24?u=pb66)

- [ADC Tutorial Part2, fixing the Makefile and flashing an image](https://community.openenergymonitor.org/t/stm32-development/6815/38?u=pb66)

- [ADC Tutorial Part3, adding user code to make it useful.](https://community.openenergymonitor.org/t/stm32-development/6815/40?u=pb66)

additionally there are the following edits to remove a couple of compile errors

- in `Inc/main.h` 2 additional include lines are required (see [forum post #44](https://community.openenergymonitor.org/t/stm32-development/6815/44?u=pb66))
    ```
    /* USER CODE BEGIN Includes */

    #include <stdbool.h>
    #include <inttypes.h>

    /* USER CODE END Includes */
    ```
- in `Src/main.c` an additional declaration is required.
    ```
    /* Private variables ---------------------------------------------------------*/

    char log_buffer[100];

    /* USER CODE BEGIN PV */
    ```
    
This project has been confirmed to compile, flash and run on a nucleo-f303re.
    
It only prints a version number at start up so using the reset button should give a single line of output each time it's pressed.
    
