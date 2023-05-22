var SerialPort = require('serialport').SerialPort;
const Readline = require('@serialport/parser-readline');
const { ReadlineParser } = require('@serialport/parser-readline')
const fs = require('fs')
var path = require('path');

var port = new SerialPort({ path: '/dev/cu.usbserial-02655161', baudRate: 115200 })
const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }))
//parser.on('data', console.log)
const fileStream = fs.createWriteStream('questdata.csv');

fileStream.write('Solar,Ultra,Lidar,Temp\n');

parser.on('data', (data) => {
    fileStream.write(`${data}\n`); //write data to csv here
});

port.on('error', (err) => {
  console.error(`Error: ${err.message}`);
});
