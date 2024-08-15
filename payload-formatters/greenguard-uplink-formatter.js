  function decodeUplink(input) {
    var data = {};
    if (input.fPort == 1 && input.bytes.length == 24) {
      data.Voltage1 = ((input.bytes[0]<<24>>16 | input.bytes[1]) / 100);
      data.Current1 = ((input.bytes[2]<<24>>16 | input.bytes[3]) / 100);
      data.Voltage2 = ((input.bytes[4]<<24>>16 | input.bytes[5]) / 100);
      data.Current2 = ((input.bytes[6]<<24>>16 | input.bytes[7]) / 100);
      data.Voltage3 = ((input.bytes[8]<<24>>16 | input.bytes[9]) / 100);
      data.Current3 = ((input.bytes[10]<<24>>16 | input.bytes[11]) / 100);
      data.AirTemperture = ((input.bytes[12]<<24>>16 | input.bytes[13]) / 100);
      data.AirHumidity = ((input.bytes[14]<<24>>16 | input.bytes[15]) / 100);
      data.AirPressure = ((input.bytes[16]<<24>>16 | input.bytes[17]) / 10);
      data.SoilTemperture = ((input.bytes[18]<<24>>16 | input.bytes[19]) / 100);
      data.SoilMoisture = ((input.bytes[20]<<24>>16 | input.bytes[21]) / 10);
      data.LoadStatus = input.bytes[22];
      data.Diag = input.bytes[23];     
    }
    return {
    data: data,
    warnings: [],
    errors: []
    };
    }