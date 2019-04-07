/*
    Copyright ® 2019 Feb devMobile Software, All Rights Reserved
 
    MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE
 */
namespace devMobile.Azure.IoTHub.IoTCore.FieldGateway.LoRa
{
	using System;
#if CLOUD2DEVICE_SEND
	using System.Collections.Concurrent;
#endif
	using System.ComponentModel;
	using System.Diagnostics;
	using System.Globalization;
	using System.Linq;
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
#if ADAFRUIT_RADIO_BONNET
		private const byte ResetLine = 25;
		private const byte InterruptLine = 22;
		private Rfm9XDevice rfm9XDevice = new Rfm9XDevice(ChipSelectPin.CS1, ResetLine, InterruptLine);
#endif
		private readonly TimeSpan DeviceRestartPeriod = new TimeSpan(0, 0, 25);

		private readonly LoggingChannel logging = new LoggingChannel("devMobile Azure IotHub LoRa Field Gateway", null, new Guid("4bd2826e-54a1-4ba9-bf63-92b73ea1ac4a"));
		private ApplicationSettings applicationSettings = null;
		private DeviceClient azureIoTHubClient = null;
#if CLOUD2DEVICE_SEND
		private ConcurrentDictionary<byte[], byte[]> sendMessageQueue = new ConcurrentDictionary<byte[], byte[]>();
#endif
		private BackgroundTaskDeferral deferral = null;

		public void Run(IBackgroundTaskInstance taskInstance)
		{
			StorageFolder localFolder = ApplicationData.Current.LocalFolder;

			try
			{
				// see if the configuration file is present if not copy minimal sample one from application directory
				if ( localFolder.TryGetItemAsync(ConfigurationFilename).AsTask().Result == null)
				{
					StorageFile templateConfigurationfile = Package.Current.InstalledLocation.GetFileAsync(ConfigurationFilename).AsTask().Result;
					templateConfigurationfile.CopyAsync(localFolder, ConfigurationFilename).AsTask();
				}

				// Load the settings from configuration file exit application if missing or invalid
				StorageFile file = localFolder.GetFileAsync(ConfigurationFilename).AsTask().Result;

				applicationSettings = (JsonConvert.DeserializeObject<ApplicationSettings>(FileIO.ReadTextAsync(file).AsTask().Result));
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("JSON configuration file load failed " + ex.Message, LoggingLevel.Error);
				return;
			}

			// Log the Application build, shield information etc.
			LoggingFields applicationBuildInformation = new LoggingFields();
#if DRAGINO
			applicationBuildInformation.AddString("Shield", "DraginoLoRaGPSHat");
#endif
#if ELECROW
			applicationBuildInformation.AddString("Shield", "ElecrowRFM95IoTBoard");
#endif
#if M2M
			applicationBuildInformation.AddString("Shield", "M2M1ChannelLoRaWanGatewayShield");
#endif
#if ELECTRONIC_TRICKS
			applicationBuildInformation.AddString("Shield", "ElectronicTricksLoRaLoRaWANShield");
#endif
#if UPUTRONICS_RPIZERO_CS0
			applicationBuildInformation.AddString("Shield", "UputronicsPiZeroLoRaExpansionBoardCS0");
#endif
#if UPUTRONICS_RPIZERO_CS1
			applicationBuildInformation.AddString("Shield", "UputronicsPiZeroLoRaExpansionBoardCS1");
#endif
#if UPUTRONICS_RPIPLUS_CS0
			applicationBuildInformation.AddString("Shield", "UputronicsPiPlusLoRaExpansionBoardCS0");
#endif
#if UPUTRONICS_RPIPLUS_CS1
			applicationBuildInformation.AddString("Shield", "UputronicsPiPlusLoRaExpansionBoardCS1");
#endif
#if ADAFRUIT_RADIO_BONNET
			applicationBuildInformation.AddString("Shield", "AdafruitRadioBonnet");
#endif
#if CLOUD_DEVICE_BOND
			applicationBuildInformation.AddString("Bond", "Supported");
#else
			applicationBuildInformation.AddString("Bond", "NotSupported");
#endif
#if CLOUD_DEVICE_PUSH
			applicationBuildInformation.AddString("Push", "Supported");
#else
			applicationBuildInformation.AddString("Push", "NotSupported");
#endif
#if CLOUD_DEVICE_SEND
			applicationBuildInformation.AddString("Send", "Supported");
#else
			applicationBuildInformation.AddString("Send", "NotSupported");
#endif
			applicationBuildInformation.AddString("Timezone", TimeZoneSettings.CurrentTimeZoneDisplayName);
			applicationBuildInformation.AddString("OSVersion", Environment.OSVersion.VersionString);
			applicationBuildInformation.AddString("MachineName", Environment.MachineName);

			// This is from the application manifest 
			Package package = Package.Current;
			PackageId packageId = package.Id;
			PackageVersion version = packageId.Version;

			applicationBuildInformation.AddString("ApplicationVersion", string.Format($"{version.Major}.{version.Minor}.{version.Build}.{version.Revision}"));
			this.logging.LogEvent("Application starting", applicationBuildInformation, LoggingLevel.Information);

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
			}

			// Wire up the field gateway restart method handler
			try
			{
				azureIoTHubClient.SetMethodHandlerAsync("Restart", RestartAsync, null);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Azure IoT Hub Restart method handler setup failed " + ex.Message, LoggingLevel.Error);
				return;
			}

#if CLOUD_DEVICE_BOND
			// Wire up the bond device method handler
			try
			{
				azureIoTHubClient.SetMethodHandlerAsync("DeviceBond", this.DeviceBondAsync, null);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Azure IoT Hub Device Bond method handler setup failed " + ex.Message, LoggingLevel.Error);
				return;
			}
#endif

#if CLOUD_DEVICE_PUSH
			// Wire up the push message to device method handler
			try
			{
				this.azureIoTHubClient.SetMethodHandlerAsync("DevicePush", this.DevicePushAsync, null);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Azure IoT Hub DevicePush method handler setup failed " + ex.Message, LoggingLevel.Error);
				return;
			}
#endif

#if CLOUD_DEVICE_SEND
			// Wire up the send message to device method handler
			try
			{
				this.azureIoTHubClient.SetMethodHandlerAsync("DeviceSend", this.DeviceSendAsync, null);
			}
			catch (Exception ex)
			{
				this.logging.LogMessage("Azure IoT Hub client DeviceSend method handler setup failed " + ex.Message, LoggingLevel.Error);
				return;
			}
#endif

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
			Debug.WriteLine("Rfm9XDevice_OnTransmit");
			this.logging.LogMessage("Rfm9XDevice_OnTransmit", LoggingLevel.Information);
		}

		private async void Rfm9XDevice_OnReceive(object sender, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
#if DEBUG
			Debug.WriteLine(@"{0:HH:mm:ss}-RX From {1} PacketSnr {2:0.0} Packet RSSI {3}dBm RSSI {4}dBm = {5} byte message", DateTime.UtcNow, BitConverter.ToString(e.Address), e.PacketSnr, e.PacketRssi, e.Rssi, e.Data.Length);
#endif

#if PAYLOAD_COMA_SEPARATED_VALUES
			await PayloadCommaSeparatedValues(azureIoTHubClient, e);
#endif

#if PAYLOAD_BINARY_BINARY_CODED_DECIMAL
			await PayloadBinaryCodedDecimal(azureIoTHubClient, e);
#endif

#if PAYLOAD_BINARY_TEXT
			await PayloadText(azureIoTHubClient, e);
#endif

#if PAYLOAD_BINARY_CAYENNE_LOW_POWER_PAYLOAD
			await PayloadProcessCayenneLowPowerPayload(azureIoTHubClient, e);
#endif

#if CLOUD2DEVICE_SEND
			// see if there are any outstand messages to reply to device with
			byte[] responseMessage;

			if (sendMessageQueue.TryGetValue( e.Address, out responseMessage))
			{
				rfm9XDevice.Send(e.Address, responseMessage);
				this.logging.LogMessage("Response message sent", LoggingLevel.Information);
			}
#endif
		}

#if PAYLOAD_COMA_SEPARATED_VALUES
		private async Task PayloadCommaSeparatedValues(DeviceClient azureIoTHubClient, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
			JObject telemetryDataPoint = new JObject();
			LoggingFields processLoggingFields = new LoggingFields();
			char[] sensorReadingSeparators = { ',' };
			char[] sensorIdAndValueSeparators = { ' ' };

			processLoggingFields.AddString("PacketSNR", e.PacketSnr.ToString("F1"));
			telemetryDataPoint.Add("PacketSNR", e.PacketSnr.ToString("F1"));
			processLoggingFields.AddInt32("PacketRSSI", e.PacketRssi);
			telemetryDataPoint.Add("PacketRSSI", e.PacketRssi);
			processLoggingFields.AddInt32("RSSI", e.Rssi);
			telemetryDataPoint.Add("RSSI", e.Rssi);

			string addressBcdText = BitConverter.ToString(e.Address);
			processLoggingFields.AddInt32("DeviceAddressLength", e.Address.Length);
			processLoggingFields.AddString("DeviceAddressBCD", addressBcdText);
			telemetryDataPoint.Add("DeviceAddressBCD", addressBcdText);

			string messageText;
			try
			{
				messageText = UTF8Encoding.UTF8.GetString(e.Data);
				processLoggingFields.AddString("MessageText", messageText);
			}
			catch (Exception)
			{
				this.logging.LogEvent("PayloadProcess failure converting payload to text", processLoggingFields, LoggingLevel.Warning);
				return;
			}

			// Chop up the CSV text
			string[] sensorReadings = messageText.Split(sensorReadingSeparators, StringSplitOptions.RemoveEmptyEntries);
			if (sensorReadings.Length < 1)
			{
				this.logging.LogEvent("PayloadProcess payload contains no sensor readings", processLoggingFields, LoggingLevel.Warning);
				return;
			}

			// Chop up each sensor read into an ID & value
			foreach (string sensorReading in sensorReadings)
			{
				string[] sensorIdAndValue = sensorReading.Split(sensorIdAndValueSeparators, StringSplitOptions.RemoveEmptyEntries);

				// Check that there is an id & value
				if (sensorIdAndValue.Length != 2)
				{
					this.logging.LogEvent("PayloadProcess payload invalid format", processLoggingFields, LoggingLevel.Warning);
					return;
				}

				string sensorId = sensorIdAndValue[0].ToLower();
				string value = sensorIdAndValue[1];

				// Construct the sensor ID from SensordeviceID & Value, mainly intended for Azure IOT Central support
				if (this.applicationSettings.SensorIDIsDeviceIDSensorID)
				{
					sensorId = string.Concat(addressBcdText, sensorId);
				}

				try
				{
					telemetryDataPoint.Add(sensorId, value);
					processLoggingFields.AddString(sensorId, value);
					Debug.WriteLine($" SensorID {sensorId} Value {value}");
				}
				catch (Exception ex)
				{
					processLoggingFields.AddString("Exception", ex.ToString());
					this.logging.LogEvent("PayloadProcess sensor reading invalid JSON format", processLoggingFields, LoggingLevel.Warning);
					return;
				}
			}

			try
			{
				using (Message message = new Message(Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(telemetryDataPoint))))
				{
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync start", DateTime.UtcNow);
					await this.azureIoTHubClient.SendEventAsync(message);
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync finish", DateTime.UtcNow);
				}
				this.logging.LogEvent("SendEventAsync CSV payload", processLoggingFields, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				processLoggingFields.AddString("Exception", ex.ToString());
				this.logging.LogEvent("SendEventAsync CSV payload", processLoggingFields, LoggingLevel.Error);
			}
		}
#endif

#if PAYLOAD_TEXT
		async Task PayloadBinaryCodedDecimal(DeviceClient azureIoTHubClient, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
			JObject telemetryDataPoint = new JObject();
			LoggingFields processLoggingFields = new LoggingFields();

			processLoggingFields.AddString("PacketSNR", e.PacketSnr.ToString("F1"));
			telemetryDataPoint.Add("PacketSNR", e.PacketSnr.ToString("F1"));
			processLoggingFields.AddInt32("PacketRSSI", e.PacketRssi);
			telemetryDataPoint.Add("PacketRSSI", e.PacketRssi);
			processLoggingFields.AddInt32("RSSI", e.Rssi);
			telemetryDataPoint.Add("RSSI", e.Rssi);

			string addressBcdText = BitConverter.ToString(e.Address);
			processLoggingFields.AddInt32("DeviceAddressLength", e.Address.Length);
			processLoggingFields.AddString("DeviceAddressBCD", addressBcdText);
			telemetryDataPoint.Add("DeviceAddressBCD", addressBcdText);

			string messageBcdText = BitConverter.ToString(e.Data);
			processLoggingFields.AddInt32("MessageLength", e.Data.Length);
			processLoggingFields.AddString("MessageBCD", messageBcdText);

			try
			{
				string messageText = UTF8Encoding.UTF8.GetString(e.Data);
				processLoggingFields.AddString("MessageText", messageText);
				telemetryDataPoint.Add("Payload", messageText);
			}
			catch (Exception)
			{
				this.logging.LogEvent("PayloadProcess failure converting payload to text", processLoggingFields, LoggingLevel.Warning);
				return;
			}

			try
			{
				using (Message message = new Message(Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(telemetryDataPoint))))
				{
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync start", DateTime.UtcNow);
					await this.azureIoTHubClient.SendEventAsync(message);
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync finish", DateTime.UtcNow);
				}
				this.logging.LogEvent("SendEventAsync BCD payload", processLoggingFields, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				processLoggingFields.AddString("Exception", ex.ToString());
				this.logging.LogEvent("SendEventAsync BCD payload", processLoggingFields, LoggingLevel.Error);
			}
		}
#endif

#if PAYLOAD_BINARY_BINARY_CODED_DECIMAL
		async Task PayloadText(DeviceClient azureIoTHubClient, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
			JObject telemetryDataPoint = new JObject();
			LoggingFields processLoggingFields = new LoggingFields();

			processLoggingFields.AddString("PacketSNR", e.PacketSnr.ToString("F1"));
			telemetryDataPoint.Add("PacketSNR", e.PacketSnr.ToString("F1"));
			processLoggingFields.AddInt32("PacketRSSI", e.PacketRssi);
			telemetryDataPoint.Add("PacketRSSI", e.PacketRssi);
			processLoggingFields.AddInt32("RSSI", e.Rssi);
			telemetryDataPoint.Add("RSSI", e.Rssi);

			string addressBcdText = BitConverter.ToString(e.Address);
			processLoggingFields.AddInt32("DeviceAddressLength", e.Address.Length);
			processLoggingFields.AddString("DeviceAddressBCD", addressBcdText);
			telemetryDataPoint.Add("DeviceAddressBCD", addressBcdText);

			string messageBcdText = BitConverter.ToString(e.Data);
			processLoggingFields.AddInt32("MessageLength", e.Data.Length);
			processLoggingFields.AddString("MessageBCD", messageBcdText);
			telemetryDataPoint.Add("Payload", messageBcdText);

			try
			{
				using (Message message = new Message(Encoding.ASCII.GetBytes(JsonConvert.SerializeObject(telemetryDataPoint))))
				{
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync start", DateTime.UtcNow);
					await this.azureIoTHubClient.SendEventAsync(message);
					Debug.WriteLine(" {0:HH:mm:ss} AzureIoTHubClient SendEventAsync finish", DateTime.UtcNow);
				}
				this.logging.LogEvent("SendEventAsync BCD payload", processLoggingFields, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				processLoggingFields.AddString("Exception", ex.ToString());
				this.logging.LogEvent("SendEventAsync Text payload", processLoggingFields, LoggingLevel.Error);
			}
		}
#endif

#if PAYLOAD_BINARY_CAYENNE_LOW_POWER_PAYLOAD
		void PayloadProcessCayenneLowPowerPayload(DeviceClient azureIoTHubClient, Rfm9XDevice.OnDataReceivedEventArgs e)
		{
		}
#endif

		private async Task<MethodResponse> RestartAsync(MethodRequest methodRequest, object userContext)
		{
			LoggingFields restartLoggingInfo = new LoggingFields();

#if CLOUD2DEVICE_SEND
			restartLoggingInfo.AddInt32("Send message queue count", sendMessageQueue.Count);
#endif
			restartLoggingInfo.AddTimeSpan("Device restart period", DeviceRestartPeriod);

			// Disconnect the transmit and receive callbacks, once messages Send & Push need to consider what todo with queued outbound messages
			rfm9XDevice.OnReceive -= Rfm9XDevice_OnReceive;
			rfm9XDevice.OnTransmit -= Rfm9XDevice_OnTransmit;

			this.logging.LogEvent("Restart initiated", restartLoggingInfo, LoggingLevel.Information);

			ShutdownManager.BeginShutdown(ShutdownKind.Restart, DeviceRestartPeriod);

			return new MethodResponse( 200 );
		}

#if CLOUD_DEVICE_BOND
		private async Task<MethodResponse> DeviceBondAsync(MethodRequest methodRequest, object userContext)
		{
			LoggingFields bondLoggingInfo = new LoggingFields();

			try
			{
				dynamic json = JValue.Parse(methodRequest.DataAsJson);

				string deviceAddressBcd = json.DeviceAddress;
				bondLoggingInfo.AddString("DeviceAddressBCD", deviceAddressBcd);
				Debug.WriteLine($"DeviceBondAsync DeviceAddressBCD {deviceAddressBcd}");

				byte[] deviceAddressBytes = deviceAddressBcd.Split('-').Select(x => byte.Parse(x, NumberStyles.HexNumber)).ToArray();
				bondLoggingInfo.AddInt32("DeviceAddressBytes Length", deviceAddressBytes.Length);
				Debug.WriteLine($"DeviceBondAsync DeviceAddressLength {deviceAddressBytes.Length}");

				if ((deviceAddressBytes.Length < Rfm9XDevice.AddressLengthMinimum) || (deviceAddressBytes.Length > Rfm9XDevice.AddressLengthMaximum))
				{
					this.logging.LogEvent("DeviceBondAsync failed device address bytes length", bondLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(414);
				}

				// Empty payload for bond message
				byte[] payloadBytes = {};

				rfm9XDevice.Send(deviceAddressBytes, payloadBytes);

				this.logging.LogEvent("DeviceBondAsync success", bondLoggingInfo, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				bondLoggingInfo.AddString("Exception", ex.ToString());
				this.logging.LogEvent("DeviceBondAsync exception", bondLoggingInfo, LoggingLevel.Error);
				return new MethodResponse(400);
			}

			return new MethodResponse(200);
		}
#endif

#if CLOUD2DEVICE_SEND
		private async Task<MethodResponse> DeviceSendAsync(MethodRequest methodRequest, object userContext)
		{
			LoggingFields sendLoggingInfo = new LoggingFields();

			this.logging.LogEvent("Send BCD initiated");

			try
			{
				// Initially use a dynamic maybe use a decorated class in future
				dynamic json = JValue.Parse(methodRequest.DataAsJson);

				string deviceAddressBcd = json.DeviceAddress;
				sendLoggingInfo.AddString("DeviceAddressBCD", deviceAddressBcd);
				Debug.WriteLine($"DeviceSendAsync DeviceAddressBCD {deviceAddressBcd}");

				byte[] deviceAddressBytes = deviceAddressBcd.Split('-').Select(x => byte.Parse(x, NumberStyles.HexNumber)).ToArray();
				sendLoggingInfo.AddInt32("DeviceAddressBytes Length", deviceAddressBytes.Length);
				Debug.WriteLine($"DeviceSendAsync DeviceAddressLength {deviceAddressBytes.Length}");

				if ((deviceAddressBytes.Length < Rfm9XDevice.AddressLengthMinimum) || (deviceAddressBytes.Length > Rfm9XDevice.AddressLengthMaximum))
				{
					this.logging.LogEvent("DeviceSendAsync failed device address bytes length", sendLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(414);
				}

				string messagedBcd = json.DevicePayload;
				sendLoggingInfo.AddString("MessageBCD", messagedBcd);

				byte[] messageBytes = messagedBcd.Split('-').Select(x => byte.Parse(x, NumberStyles.HexNumber)).ToArray(); // changed the '-' to ' '
				sendLoggingInfo.AddInt32("MessageBytesLength", messageBytes.Length);
				Debug.WriteLine($"DeviceSendAsync DeviceAddress:{deviceAddressBcd} Payload:{messagedBcd}");

				if ((messageBytes.Length < Rfm9XDevice.MessageLengthMinimum) || (messageBytes.Length > Rfm9XDevice.MessageLengthMaximum))
				{
					this.logging.LogEvent("DeviceSendAsync failed payload Length", sendLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(413);
				}

				if ( sendMessageQueue.TryAdd(deviceAddressBytes, messageBytes))
				{
					this.logging.LogEvent("DeviceSendAsync failed message already queued", sendLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(409);
				}

				this.logging.LogEvent("DeviceSendAsync success", sendLoggingInfo, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				sendLoggingInfo.AddString("Exception", ex.ToString());
				this.logging.LogEvent("DeviceSendAsync failed exception", sendLoggingInfo, LoggingLevel.Error);
				return new MethodResponse(400);
			}

			return new MethodResponse(200);
		}
#endif

#if CLOUD2DEVICE_PUSH
		private async Task<MethodResponse> DevicePushAsync(MethodRequest methodRequest, object userContext)
		{
			LoggingFields pushLoggingInfo = new LoggingFields();

			this.logging.LogEvent("Push BCD initiated");

			try
			{
				// Initially use a dynamic maybe use a decorated class in future
				dynamic json = JValue.Parse(methodRequest.DataAsJson);

				string deviceAddressBcd = json.DeviceAddress;
				pushLoggingInfo.AddString("DeviceAddressBCD", deviceAddressBcd);
				Debug.WriteLine($"DevicePushAsync DeviceAddressBCD {deviceAddressBcd}");

				byte[] deviceAddressBytes = deviceAddressBcd.Split('-').Select(x => byte.Parse(x, NumberStyles.HexNumber)).ToArray();
				pushLoggingInfo.AddInt32("DeviceAddressBytes Length", deviceAddressBytes.Length);
				Debug.WriteLine($"DevicePushAsync DeviceAddressLength {deviceAddressBytes.Length}");

				if ((deviceAddressBytes.Length < Rfm9XDevice.AddressLengthMinimum) || (deviceAddressBytes.Length > Rfm9XDevice.AddressLengthMaximum))
				{
					this.logging.LogEvent("DevicePushAsync failed device address bytes length", pushLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(414);
				}

				string messagedBcd = json.DevicePayload;
				pushLoggingInfo.AddString("MessageBCD", messagedBcd);

				byte[] messageBytes = messagedBcd.Split('-').Select(x => byte.Parse(x, NumberStyles.HexNumber)).ToArray(); // changed the '-' to ' '
				pushLoggingInfo.AddInt32("MessageBytesLength", messageBytes.Length);
				Debug.WriteLine($"DevicePushAsync DeviceAddress:{deviceAddressBcd} Payload:{messagedBcd}");

				if ((messageBytes.Length < Rfm9XDevice.MessageLengthMinimum) || (messageBytes.Length > Rfm9XDevice.MessageLengthMaximum))
				{
					this.logging.LogEvent("DevicePushAsync failed payload Length", pushLoggingInfo, LoggingLevel.Error);
					return new MethodResponse(413);
				}

				rfm9XDevice.Send(deviceAddressBytes, messageBytes);

				this.logging.LogEvent("DevicePushAsync success", pushLoggingInfo, LoggingLevel.Information);
			}
			catch (Exception ex)
			{
				pushLoggingInfo.AddString("Exception", ex.ToString());
				this.logging.LogEvent("DevicePushAsync failed exception", pushLoggingInfo, LoggingLevel.Error);
				return new MethodResponse(400);
			}

			return new MethodResponse(200);
		}
#endif

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
