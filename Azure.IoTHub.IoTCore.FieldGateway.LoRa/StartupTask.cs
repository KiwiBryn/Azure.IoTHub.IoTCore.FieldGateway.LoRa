//---------------------------------------------------------------------------------
// Copyright ® September 2018, devMobile Software
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//---------------------------------------------------------------------------------
namespace devMobile.Azure.IoTHub.IoTCore.FieldGateway.LoRa
{
	using System;
	using System.ComponentModel;
	using System.Diagnostics;
	using System.IO;
	using System.Text;
	using System.Threading.Tasks;

	using devMobile.IoT.Rfm9x;

	using Microsoft.Azure.Devices.Client;
	using Microsoft.Azure.Devices.Shared;
	using Newtonsoft.Json;
	using Newtonsoft.Json.Converters;
	using Newtonsoft.Json.Linq;
	using Windows.ApplicationModel;
	using Windows.ApplicationModel.Background;
	using Windows.Foundation.Diagnostics;
	using Windows.Storage;
	using Windows.Storage.Streams;
	using Windows.System;
	using Windows.System.Profile;

	public sealed class StartupTask : IBackgroundTask
	{
		private const string ConfigurationFilename = "config.json";

		// LoRa Hardware interface configuration
#if DRAGINO
		private const byte ChipSelectLine = 25;
		private const byte ResetLine = 17;
		private const byte InterruptLine = 4;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS0, ChipSelectLine, ResetLine, InterruptLine);
#endif
#if M2M
		private const byte ChipSelectLine = 25;
		private const byte ResetLine = 17;
		private const byte InterruptLine = 4;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS0, ChipSelectLine, ResetLine, InterruptLine);
#endif
#if ELECROW
		private const byte ResetLine = 22;
		private const byte InterruptLine = 25;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS1, ResetLine, InterruptLine);
#endif
#if ELECTRONIC_TRICKS
		private const byte ResetLine = 22;
		private const byte InterruptLine = 25;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS0, ResetLine, InterruptLine);
#endif
#if UPUTRONICS_RPIZERO_CS0
		private const byte InterruptLine = 25;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS0, InterruptLine);
#endif
#if UPUTRONICS_RPIZERO_CS1
		private const byte InterruptLine = 16;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS1, InterruptLine);
#endif
#if UPUTRONICS_RPIPLUS_CS0
		private const byte InterruptLine = 25;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS0, InterruptLine);
#endif
#if UPUTRONICS_RPIPLUS_CS1 
		private const byte InterruptLine = 16;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS1, InterruptLine);
#endif
		private const byte AddressLengthMinimum = 1;
		private const byte AddressLengthMaximum = 15;

		private const byte MessageLengthMinimum = 3;
		private const byte MessageLengthMaximum = 128;

		private readonly TimeSpan DeviceRestartPeriod = new TimeSpan(0, 0, 25);

		private readonly LoggingChannel logging = new LoggingChannel("devMobile Azure IotHub LoRa Field Gateway", null, new Guid("4bd2826e-54a1-4ba9-bf63-92b73ea1ac4a"));
		private ApplicationSettings applicationSettings = null;
		private DeviceClient azureIoTHubClient = null;
		private BackgroundTaskDeferral deferral = null;

		public void Run(IBackgroundTaskInstance taskInstance)
		{

			if (!this.ConfigurationFileLoad().Result)
			{
				return;
			}

			// Log the Application build, shield information etc.
			LoggingFields appllicationBuildInformation = new LoggingFields();
#if DRAGINO
			appllicationBuildInformation.AddString("Shield", "DraginoLoRaGPSHat");
#endif
#if ELECROW
			appllicationBuildInformation.AddString("Shield", "ElecrowRFM95IoTBoard");
#endif
#if M2M
			appllicationBuildInformation.AddString("Shield", "M2M1ChannelLoRaWanGatewayShield");
#endif
#if ELECTRONIC_TRICKS
			appllicationBuildInformation.AddString("Shield", "ElectronicTricksLoRaLoRaWANShield");
#endif
#if UPUTRONICS_RPIZERO_CS0
			appllicationBuildInformation.AddString("Shield", "UputronicsPiZeroLoRaExpansionBoardCS0");
#endif
#if UPUTRONICS_RPIZERO_CS1
			appllicationBuildInformation.AddString("Shield", "UputronicsPiZeroLoRaExpansionBoardCS1");
#endif
#if UPUTRONICS_RPIPLUS_CS0
			appllicationBuildInformation.AddString("Shield", "UputronicsPiPlusLoRaExpansionBoardCS0");
#endif
#if UPUTRONICS_RPIPLUS_CS1
			appllicationBuildInformation.AddString("Shield", "UputronicsPiPlusLoRaExpansionBoardCS1");
#endif
			appllicationBuildInformation.AddString("Timezone", TimeZoneSettings.CurrentTimeZoneDisplayName);
			appllicationBuildInformation.AddString("OSVersion", Environment.OSVersion.VersionString);
			appllicationBuildInformation.AddString("MachineName", Environment.MachineName);

			// This is from the application manifest 
			Package package = Package.Current;
			PackageId packageId = package.Id;
			PackageVersion version = packageId.Version;

			appllicationBuildInformation.AddString("ApplicationVersion", string.Format($"{version.Major}.{version.Minor}.{version.Build}.{version.Revision}"));
			this.logging.LogEvent("Application starting", appllicationBuildInformation, LoggingLevel.Information);

			// Log the Azure connection string and associated settings
			LoggingFields azureIoTHubSettings = new LoggingFields();
			azureIoTHubSettings.AddString("DeviceConnectionString", this.applicationSettings.AzureIoTHubDeviceConnectionString);
			azureIoTHubSettings.AddString("TransportType", this.applicationSettings.AzureIoTHubTransportType.ToString());
			azureIoTHubSettings.AddString("SensorIDIsDeviceIDSensorID", this.applicationSettings.SensorIDIsDeviceIDSensorID.ToString());
			this.logging.LogEvent("AzureIoTHub configuration", azureIoTHubSettings, LoggingLevel.Information);

			// Connect the IoT hub first so we are ready for any messages
			try
			{
				this.azureIoTHubClient = DeviceClient.CreateFromConnectionString(this.applicationSettings.AzureIoTHubDeviceConnectionString, this.applicationSettings.AzureIoTHubTransportType);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("IoT Hub connection failed " + ex.Message, LoggingLevel.Error);
				return;
			}

			try
			{
				TwinCollection reportedProperties;
				reportedProperties = new TwinCollection();

				// This is from the OS 
				reportedProperties["Timezone"] = TimeZoneSettings.CurrentTimeZoneDisplayName;
				reportedProperties["OSVersion"] = Environment.OSVersion.VersionString;
				reportedProperties["MachineName"] = Environment.MachineName;

				reportedProperties["ApplicationDisplayName"] = package.DisplayName;
				reportedProperties["ApplicationName"] = packageId.Name;
				reportedProperties["ApplicationVersion"] = string.Format($"{version.Major}.{version.Minor}.{version.Build}.{version.Revision}");

				// Unique identifier from the hardware
				SystemIdentificationInfo systemIdentificationInfo = SystemIdentification.GetSystemIdForPublisher();
				using (DataReader reader = DataReader.FromBuffer(systemIdentificationInfo.Id))
				{
					byte[] bytes = new byte[systemIdentificationInfo.Id.Length];
					reader.ReadBytes(bytes);
					reportedProperties["SystemId"] = BitConverter.ToString(bytes);
				}

				azureIoTHubClient.UpdateReportedPropertiesAsync(reportedProperties).Wait();
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("IoT Hub updating reported properties failed " + ex.Message, LoggingLevel.Error);
				return;
			}

			try
			{
				azureIoTHubClient.SetMethodHandlerAsync("Restart", RestartAsync, null);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Azure IoT Hub Restart method handler setup failed " + ex.Message, LoggingLevel.Error);
				return;
			}

			// Configure the LoRa module
			rfm9XDevice.OnReceive += Rfm9XDevice_OnReceive;
			rfm9XDevice.OnTransmit += Rfm9XDevice_OnTransmit;

			rfm9XDevice.Initialise(this.applicationSettings.Frequency,
				rxDoneignoreIfCrcMissing: true,
				rxDoneignoreIfCrcInvalid: true,
				paBoost: this.applicationSettings.PABoost, maxPower: this.applicationSettings.MaxPower, outputPower: this.applicationSettings.OutputPower,
				ocpOn: this.applicationSettings.OCPOn, ocpTrim: this.applicationSettings.OCPTrim,
				lnaGain: this.applicationSettings.LnaGain, lnaBoost: this.applicationSettings.LNABoost,
				bandwidth: this.applicationSettings.Bandwidth, codingRate: this.applicationSettings.CodingRate, implicitHeaderModeOn: this.applicationSettings.ImplicitHeaderModeOn,
				spreadingFactor: this.applicationSettings.SpreadingFactor,
				rxPayloadCrcOn: true,
				symbolTimeout: this.applicationSettings.SymbolTimeout,
				preambleLength: this.applicationSettings.PreambleLength,
				payloadLength: this.applicationSettings.PayloadLength,
				payloadMaxLength: this.applicationSettings.PayloadMaxLength,
				freqHoppingPeriod: this.applicationSettings.FreqHoppingPeriod,
				lowDataRateOptimize: this.applicationSettings.LowDataRateOptimize,
				ppmCorrection: this.applicationSettings.PpmCorrection,
				detectionOptimize: this.applicationSettings.DetectionOptimize,
				invertIQ: this.applicationSettings.InvertIQ,
				detectionThreshold: this.applicationSettings.DetectionThreshold,
				syncWord: this.applicationSettings.SyncWord
				);

#if DEBUG
			rfm9XDevice.RegisterDump();
#endif

			rfm9XDevice.Receive(Encoding.UTF8.GetBytes(this.applicationSettings.Address));

			LoggingFields loRaSettings = new LoggingFields();
			loRaSettings.AddString("Address", this.applicationSettings.Address);
			loRaSettings.AddDouble("Frequency", this.applicationSettings.Frequency);
			loRaSettings.AddBoolean("PABoost", this.applicationSettings.PABoost);

			loRaSettings.AddUInt8("MaxPower", this.applicationSettings.MaxPower);
			loRaSettings.AddUInt8("OutputPower", this.applicationSettings.OutputPower);
			loRaSettings.AddBoolean("OCPOn", this.applicationSettings.OCPOn);
			loRaSettings.AddUInt8("OCPTrim", this.applicationSettings.OCPTrim);

			loRaSettings.AddString("LnaGain", this.applicationSettings.LnaGain.ToString());
			loRaSettings.AddBoolean("lnaBoost", this.applicationSettings.LNABoost);

			loRaSettings.AddString("codingRate", this.applicationSettings.CodingRate.ToString());
			loRaSettings.AddString("implicitHeaderModeOn", applicationSettings.ImplicitHeaderModeOn.ToString());
			loRaSettings.AddString("spreadingFactor", this.applicationSettings.SpreadingFactor.ToString());
			loRaSettings.AddBoolean("rxPayloadCrcOn", true);

			loRaSettings.AddUInt8("symbolTimeout", this.applicationSettings.SymbolTimeout);
			loRaSettings.AddUInt8("preambleLength", this.applicationSettings.PreambleLength);
			loRaSettings.AddUInt8("payloadLength", this.applicationSettings.PayloadLength);

			loRaSettings.AddUInt8("payloadMaxLength", this.applicationSettings.PayloadMaxLength);
			loRaSettings.AddUInt8("freqHoppingPeriod", this.applicationSettings.FreqHoppingPeriod);
			loRaSettings.AddBoolean("lowDataRateOptimize", this.applicationSettings.LowDataRateOptimize);
			loRaSettings.AddUInt8("ppmCorrection", this.applicationSettings.PpmCorrection);

			loRaSettings.AddString("detectionOptimize", this.applicationSettings.DetectionOptimize.ToString());
			loRaSettings.AddBoolean("invertIQ", this.applicationSettings.InvertIQ);
			loRaSettings.AddString("detectionThreshold", this.applicationSettings.DetectionThreshold.ToString());
			loRaSettings.AddUInt8("SyncWord", this.applicationSettings.SyncWord);

			this.logging.LogEvent("LoRa configuration", loRaSettings, LoggingLevel.Information);

			this.deferral = taskInstance.GetDeferral();
		}

		private void Rfm9XDevice_OnTransmit(object sender, Rfm9XDevice.OnDataTransmitedEventArgs e)
		{
			throw new NotImplementedException();
		}

		private async Task<bool> ConfigurationFileLoad()
		{
			StorageFolder localFolder = ApplicationData.Current.LocalFolder;

			try
			{
				// Check to see if file exists
				if (localFolder.TryGetItemAsync(ConfigurationFilename).GetAwaiter().GetResult() == null)
				{
					this.logging.LogMessage("Configuration file " + ConfigurationFilename + " not found", LoggingLevel.Error);

					this.applicationSettings = new ApplicationSettings()
					{
						AzureIoTHubDeviceConnectionString = "Azure IoT Hub connection string goes here",
						AzureIoTHubTransportType = TransportType.Amqp,
						Address = "Address here",
						Frequency = 915000000,
					};

					// Create empty configuration file
					StorageFile configurationFile = await localFolder.CreateFileAsync(ConfigurationFilename, CreationCollisionOption.OpenIfExists);
					using (Stream stream = await configurationFile.OpenStreamForWriteAsync())
					{
						using (TextWriter streamWriter = new StreamWriter(stream))
						{
							streamWriter.Write(JsonConvert.SerializeObject(this.applicationSettings, Formatting.Indented));
						}
					}

					return false;
				}
				else
				{
					// Load the configuration settings
					StorageFile configurationFile = await localFolder.CreateFileAsync(ConfigurationFilename, CreationCollisionOption.OpenIfExists);
					using (Stream stream = await configurationFile.OpenStreamForReadAsync())
					{
						using (TextReader streamReader = new StreamReader(stream))
						{
							this.applicationSettings = JsonConvert.DeserializeObject<ApplicationSettings>(streamReader.ReadToEnd());
						}
					}

					return true;
				}
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Configuration file " + ConfigurationFilename + " load failed " + ex.Message, LoggingLevel.Error);
				return false;
			}
		}

		private async void Rfm9XDevice_OnReceive(object sender, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
			string addressBcdText;
			string messageBcdText;
			string messageText = "";
			char[] sensorReadingSeparators = new char[] { ',' };
			char[] sensorIdAndValueSeparators = new char[] { ' ' };

			addressBcdText = BitConverter.ToString(e.Address).ToLower();

			messageBcdText = BitConverter.ToString(e.Data);
			try
			{
				messageText = UTF8Encoding.UTF8.GetString(e.Data);
			}
			catch (Exception)
			{
				this.logging.LogMessage("Failure converting payload to text", LoggingLevel.Error);
				return;
			}


#if DEBUG
			Debug.WriteLine(@"{0:HH:mm:ss}-RX From {1} PacketSnr {2:0.0} Packet RSSI {3}dBm RSSI {4}dBm = {5} byte message ""{6}""", DateTime.Now, addressBcdText, e.PacketSnr, e.PacketRssi, e.Rssi, e.Data.Length, messageText);
#endif
			LoggingFields messagePayload = new LoggingFields();
			messagePayload.AddInt32("AddressLength", e.Address.Length);
			messagePayload.AddString("Address-BCD", addressBcdText);
			messagePayload.AddInt32("Message-Length", e.Data.Length);
			messagePayload.AddString("Message-BCD", messageBcdText);
			messagePayload.AddString("Message-Unicode", messageText);
			messagePayload.AddDouble("Packet SNR", e.PacketSnr);
			messagePayload.AddInt32("Packet RSSI", e.PacketRssi);
			messagePayload.AddInt32("RSSI", e.Rssi);
			this.logging.LogEvent("Message Data", messagePayload, LoggingLevel.Verbose);

			// Check the address is not to short/long 
			if (e.Address.Length < AddressLengthMinimum)
			{
				this.logging.LogMessage("From address too short", LoggingLevel.Warning);
				return;
			}

			if (e.Address.Length > MessageLengthMaximum)
			{
				this.logging.LogMessage("From address too long", LoggingLevel.Warning);
				return;
			}

			// Check the payload is not too short/long 
			if (e.Data.Length < MessageLengthMinimum)
			{
				this.logging.LogMessage("Message too short to contain any data", LoggingLevel.Warning);
				return;
			}

			if (e.Data.Length > MessageLengthMaximum)
			{
				this.logging.LogMessage("Message too long to contain valid data", LoggingLevel.Warning);
				return;
			}

			// Chop up the CSV text
			string[] sensorReadings = messageText.Split(sensorReadingSeparators, StringSplitOptions.RemoveEmptyEntries);
			if (sensorReadings.Length < 1)
			{
				this.logging.LogMessage("Payload contains no sensor readings", LoggingLevel.Warning);
				return;
			}

			JObject telemetryDataPoint = new JObject(); // This could be simplified but for field gateway will use this style
			LoggingFields sensorData = new LoggingFields();

			telemetryDataPoint.Add("DeviceID", addressBcdText);
			sensorData.AddString("DeviceID", addressBcdText);
			telemetryDataPoint.Add("PacketSNR", e.PacketSnr.ToString("F1"));
			sensorData.AddString("PacketSNR", e.PacketSnr.ToString("F1"));
			telemetryDataPoint.Add("PacketRSSI", e.PacketRssi);
			sensorData.AddInt32("PacketRSSI", e.PacketRssi);
			telemetryDataPoint.Add("RSSI", e.Rssi);
			sensorData.AddInt32("RSSI", e.Rssi);

			// Chop up each sensor read into an ID & value
			foreach (string sensorReading in sensorReadings)
			{
				string[] sensorIdAndValue = sensorReading.Split(sensorIdAndValueSeparators, StringSplitOptions.RemoveEmptyEntries);

				// Check that there is an id & value
				if (sensorIdAndValue.Length != 2)
				{
					this.logging.LogMessage("Sensor reading invalid format", LoggingLevel.Warning);
					return;
				}

				string sensorId = sensorIdAndValue[0].ToLower();
				string value = sensorIdAndValue[1];

				try
				{
					if (this.applicationSettings.SensorIDIsDeviceIDSensorID)
					{
						// Construct the sensor ID from SensordeviceID & Value ID
						telemetryDataPoint.Add(string.Format("{0}{1}", addressBcdText, sensorId), value);

						sensorData.AddString(string.Format("{0}{1}", addressBcdText, sensorId), value);
						Debug.WriteLine(" Sensor {0}{1} Value {2}", addressBcdText, sensorId, value);
					}
					else
					{
						telemetryDataPoint.Add(sensorId, value);

						sensorData.AddString(sensorId, value);
						Debug.WriteLine(" Device {0} Sensor {1} Value {2}", addressBcdText, sensorId, value);
					}
				}
				catch (Exception ex)
				{
					this.logging.LogMessage("Sensor reading invalid JSON format " + ex.Message, LoggingLevel.Warning);
					return;
				}
			}

			this.logging.LogEvent("Sensor readings", sensorData, LoggingLevel.Information);

			try
			{
				using (Message message = new Message(Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(telemetryDataPoint))))
				{
					Debug.WriteLine(" AzureIoTHubClient SendEventAsync start");
					await this.azureIoTHubClient.SendEventAsync(message);
					Debug.WriteLine(" AzureIoTHubClient SendEventAsync finish");
				}
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("AzureIoTHubClient SendEventAsync failed " + ex.Message, LoggingLevel.Error);
			}
		}

		private async Task<MethodResponse> RestartAsync(MethodRequest methodRequest, object userContext)
		{
			this.logging.LogMessage("Restart initiated", LoggingLevel.Information);

			// Disconnect the transmit and receive callbacks, once messages Send & Push need to consider what todo with queued outbound messages
			rfm9XDevice.OnReceive -= Rfm9XDevice_OnReceive;
			rfm9XDevice.OnTransmit -= Rfm9XDevice_OnTransmit;

			ShutdownManager.BeginShutdown(ShutdownKind.Restart, DeviceRestartPeriod);

			return new MethodResponse( 200 );
		}

		private class ApplicationSettings
		{
			[JsonProperty("AzureIoTHubDeviceConnectionString", Required = Required.Always)]
			public string AzureIoTHubDeviceConnectionString { get; set; }

			[JsonProperty("AzureIoTHubTransportType", Required = Required.Always)]
			[JsonConverter(typeof(StringEnumConverter))]
			public TransportType AzureIoTHubTransportType { get; internal set; }

			[JsonProperty("SensorIDIsDeviceIDSensorID", Required = Required.Always)]
			public bool SensorIDIsDeviceIDSensorID { get; set; }


			// LoRa configuration parameters
			[JsonProperty("Address", Required = Required.Always)]
			public string Address { get; set; }

			[DefaultValue(Rfm9XDevice.FrequencyDefault)]
			[JsonProperty("Frequency", DefaultValueHandling = DefaultValueHandling.Populate)]
			public double Frequency { get; set; }

			// RegPaConfig
			[DefaultValue(Rfm9XDevice.PABoostDefault)]
			[JsonProperty("PABoost", DefaultValueHandling = DefaultValueHandling.Populate)]
			public bool PABoost { get; set; }

			[DefaultValue(Rfm9XDevice.RegPAConfigMaxPowerDefault)]
			[JsonProperty("MaxPower", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte MaxPower { get; set; }

			[DefaultValue(Rfm9XDevice.RegPAConfigOutputPowerDefault)]
			[JsonProperty("OutputPower", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte OutputPower { get; set; }

			// RegOcp
			[DefaultValue(Rfm9XDevice.RegOcpDefault)]
			[JsonProperty("OCPOn", DefaultValueHandling = DefaultValueHandling.Populate)]
			public bool OCPOn { get; set; }

			[DefaultValue(Rfm9XDevice.RegOcpOcpTrimDefault)]
			[JsonProperty("OCPTrim", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte OCPTrim { get; set; }

			// RegLna
			[DefaultValue(Rfm9XDevice.LnaGainDefault)]
			[JsonProperty("LNAGain", DefaultValueHandling = DefaultValueHandling.Populate)]
			[JsonConverter(typeof(StringEnumConverter))]
			public Rfm9XDevice.RegLnaLnaGain LnaGain { get; set; }

			[DefaultValue(Rfm9XDevice.LnaBoostDefault)]
			[JsonProperty("LNABoost", DefaultValueHandling = DefaultValueHandling.Populate)]
			public bool LNABoost { get; set; }

			// RegModemConfig1
			[DefaultValue(Rfm9XDevice.RegModemConfigBandwidthDefault)]
			[JsonProperty("Bandwidth", DefaultValueHandling = DefaultValueHandling.Populate)]
			[JsonConverter(typeof(StringEnumConverter))]
			public Rfm9XDevice.RegModemConfigBandwidth Bandwidth { get; set; }

			[DefaultValue(Rfm9XDevice.RegModemConfigCodingRateDefault)]
			[JsonProperty("codingRate", DefaultValueHandling = DefaultValueHandling.Populate)]
			[JsonConverter(typeof(StringEnumConverter))]
			public Rfm9XDevice.RegModemConfigCodingRate CodingRate { get; set; }

			[DefaultValue(Rfm9XDevice.RegModemConfigImplicitHeaderModeOnDefault)]
			[JsonProperty("ImplicitHeaderModeOn", DefaultValueHandling = DefaultValueHandling.Populate)]
			[JsonConverter(typeof(StringEnumConverter))]
			public Rfm9XDevice.RegModemConfigImplicitHeaderModeOn ImplicitHeaderModeOn { get; set; }

			// RegModemConfig2SpreadingFactor
			[DefaultValue(Rfm9XDevice.RegModemConfig2SpreadingFactorDefault)]
			[JsonProperty("SpreadingFactor", DefaultValueHandling = DefaultValueHandling.Populate)]
			public Rfm9XDevice.RegModemConfig2SpreadingFactor SpreadingFactor { get; set; }

			[DefaultValue(Rfm9XDevice.SymbolTimeoutDefault)]
			[JsonProperty("SymbolTimeout", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte SymbolTimeout { get; set; }

			[DefaultValue(Rfm9XDevice.PreambleLengthDefault)]
			[JsonProperty("PreambleLength", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte PreambleLength { get; set; }

			[DefaultValue(Rfm9XDevice.PayloadLengthDefault)]
			[JsonProperty("PayloadLength", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte PayloadLength { get; set; }

			[DefaultValue(Rfm9XDevice.PayloadMaxLengthDefault)]
			[JsonProperty("PayloadMaxLength", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte PayloadMaxLength { get; set; }

			[DefaultValue(Rfm9XDevice.FreqHoppingPeriodDefault)]
			[JsonProperty("freqHoppingPeriod", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte FreqHoppingPeriod { get; set; }

			[DefaultValue(Rfm9XDevice.LowDataRateOptimizeDefault)]
			[JsonProperty("LowDataRateOptimize", DefaultValueHandling = DefaultValueHandling.Populate)]
			public bool LowDataRateOptimize { get; set; }

			[DefaultValue(Rfm9XDevice.AgcAutoOnDefault)]
			[JsonProperty("AgcAutoOn", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte AgcAutoOn { get; set; }

			[DefaultValue(Rfm9XDevice.ppmCorrectionDefault)]
			[JsonProperty("PPMCorrection", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte PpmCorrection { get; set; }

			[DefaultValue(Rfm9XDevice.RegDetectOptimizeDectionOptimizeDefault)]
			[JsonProperty("DetectionOptimize", DefaultValueHandling = DefaultValueHandling.Populate)]
			public Rfm9XDevice.RegDetectOptimizeDectionOptimize DetectionOptimize { get; set; }

			[DefaultValue(Rfm9XDevice.InvertIqDefault)]
			[JsonProperty("InvertIQ", DefaultValueHandling = DefaultValueHandling.Populate)]
			public bool InvertIQ { get; set; }

			[DefaultValue(Rfm9XDevice.RegisterDetectionThresholdDefault)]
			[JsonProperty("DetectionThreshold", DefaultValueHandling = DefaultValueHandling.Populate)]
			public Rfm9XDevice.RegisterDetectionThreshold DetectionThreshold { get; set; }

			[DefaultValue(Rfm9XDevice.RegSyncWordDefault)]
			[JsonProperty("SyncWord", DefaultValueHandling = DefaultValueHandling.Populate)]
			public byte SyncWord { get; set; }
		}
	}
}