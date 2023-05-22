## Code Readme

# Quest2.c

This code includes various libraries and defines constants and functions related to I2C communication, ADC readings, and GPIO configuration.

The first section includes standard C libraries and FreeRTOS, which is a real-time operating system.

The second section defines constants related to the I2C bus and initializes ADC channels for the voltage, ultrasonic, and photocell sensors. The get_voltage() function is also defined to read the voltage from the photocell sensor.

The third section includes a function to test the connection to an I2C device and a function to scan for I2C devices. The i2c_master_init() function initializes the I2C port and defines its configuration.

# Quest2.html

This is an HTML page that displays a chart of sensor readings over time. The data is fetched from a CSV file and the chart is created using the CanvasJS library. The chart displays four types of sensor readings (Lidar, Temp, Ultra, Solar) as separate lines, and includes a legend that allows the user to toggle the visibility of each line.

The chart is initially created when the page is loaded using the createChart() function. The function fetches the sensor data from the CSV file, parses it, and stores it in an object. The object is used to create the chart using the CanvasJS library. The chart object is stored in a global variable so that it can be updated later.

The chart is updated every 5 seconds using the setInterval() function and an AJAX request to fetch updated data from the server. The updated data is parsed and used to update the data points for each line in the chart. The chart is then re-rendered with the new data.

There is also a toggleDataSeries() function that allows the user to toggle the visibility of each line in the chart. This function is called when the user clicks on a legend item.

Finally, there is another setInterval() function that calls the updateChart() function every second. However, this function is never used and seems to be redundant with the previous interval set up.


# Quest2.js

This code sets up a connection with a serial port using the serialport module in Node.js. It reads data from the serial port using the parser-readline module and writes the received data to a CSV file using the fs module. The CSV file is named "questdata.csv" and contains four columns - "Solar", "Ultra", "Lidar", and "Temp". If there is an error with the serial port connection, an error message is printed to the console.


