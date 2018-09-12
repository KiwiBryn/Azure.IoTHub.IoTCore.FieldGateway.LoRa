//---------------------------------------------------------------------------------
// Copyright (c) August 2018, devMobile Software
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
namespace devMobile.IoT.Rfm9x
{
	using System;
	using System.Diagnostics;
   using System.Threading.Tasks;
	using Windows.Devices.Gpio;

	public sealed class Rfm9XDevice
	{
		public delegate void OnDataReceivedHandler(byte[] data);
#if ADDRESSED_MESSAGES_PAYLOAD
		public const byte AddressHeaderLength = 1;
		public const byte AddressLengthMinimum = 1;
		public const byte AddressLengthMaximum = 15;
#endif

		public class OnDataReceivedEventArgs : EventArgs
		{
#if ADDRESSED_MESSAGES_PAYLOAD
			public byte[] Address { get; set; }
#endif
			public float PacketSnr { get; set; }
			public int PacketRssi { get; set; }
			public int Rssi { get; set; }
			public byte[] Data { get; set; }
		}
		public delegate void onReceivedEventHandler(Object sender, OnDataReceivedEventArgs e);
		public event onReceivedEventHandler OnReceive;

		public delegate void OnDataTransmittedHandler(byte[] data);
		public class OnDataTransmitedEventArgs : EventArgs
		{
			public byte[] Data { get; set; }
		}
		public delegate void onTransmittededEventHandler(Object sender, OnDataTransmitedEventArgs e);
		public event onTransmittededEventHandler OnTransmit;


		// Registers from SemTech SX127X Datasheet
		enum Registers : byte
		{
			MinValue = RegOpMode,

			RegFifo = 0x0,
			RegOpMode = 0x01,
			//Reserved 0x02-0x06 
			RegFrMsb = 0x06,
			RegFrMid = 0x7,
			RegFrLsb = 0x08,
			RegPAConfig = 0x09,
			//RegPARamp = 0x0A, // not inlcuded as FSK/OOK functionality
			RegOcp = 0x0B,
			RegLna = 0x0C,
			RegFifoAddrPtr = 0x0D,
			RegFifoTxBaseAddr = 0x0E,
			RegFifoRxCurrent = 0x10,
			RegIrqFlagsMask = 0x11,
			RegIrqFlags = 0x12,
			RegRxNbBytes = 0x13,
			// RegRxHeaderCnValueMsb=0x14
			// RegRxHeaderCnValueLsb=0x15
			// RegRxPacketCntValueMsb=0x16
			// RegRxPacketCntValueMsb=0x17
			// RegModemStat=0x18
			RegPktSnrValue=0x19,
			RegPktRssiValue=0x1A,
			RegRssiValue=0x1B,
			RegHopChannel = 0x1C,
			RegModemConfig1 = 0x1D,
			RegModemConfig2 = 0x1E,
			RegSymbTimeout = 0x1F,
			RegPreambleMsb = 0x20,
			RegPreambleLsb = 0x21,
			RegPayloadLength = 0x22,
			RegMaxPayloadLength = 0x23,
			RegHopPeriod = 0x24,
			// RegFifiRxByteAddr = 0x25
			RegModemConfig3 = 0x26,
			RegPpmCorrection = 0x27,
			// RegFeiMsb = 0x28
			// RegFeiMid = 0x29
			// RegFeiLsb = 0x2A
			// Reserved 0x2B
			// RegRssiWideband = 0x2C
			// Reserved 0x2D-0x30
			RegDetectOptimize = 0x31,
			// Reserved 0x32
			RegInvertIQ = 0x33,
			// Reserved 0x34-0x36
			RegDetectionThreshold = 0x37,
			// Reserved 0x38
			RegSyncWord = 0x39,
			RegDioMapping1 = 0x40,
			RegVersion = 0x42,

			MaxValue = RegVersion,
		}

		// RegOpMode mode flags
		private const byte RegOpModeLongRangeModeLoRa = 0b10000000;
		private const byte RegOpModeLongRangeModeFskOok = 0b00000000;
		private const byte RegOpModeLongRangeModeDefault = RegOpModeLongRangeModeFskOok;

		private const byte RegOpModeAcessSharedRegLoRa = 0b00000000;
		private const byte RegOpModeAcessSharedRegFsk = 0b01000000;
		private const byte RegOpModeAcessSharedRegDefault = RegOpModeAcessSharedRegLoRa;

		private const byte RegOpModeLowFrequencyModeOnHighFrequency = 0b00000000;
		private const byte RegOpModeLowFrequencyModeOnLowFrequency = 0b00001000;
		private const byte RegOpModeLowFrequencyModeOnDefault = RegOpModeLowFrequencyModeOnLowFrequency;

		[Flags]
		public enum RegOpModeMode : byte
		{
			Sleep = 0b00000000,
			StandBy = 0b00000001,
			FrequencySynthesisTX = 0b00000010,
			Transmit = 0b00000011,
			FrequencySynthesisRX = 0b00000100,
			ReceiveContinuous = 0b00000101,
			ReceiveSingle = 0b00000110,
			ChannelActivityDetection = 0b00000111,
		};

		// Frequency configuration magic numbers from Semtech SX127X specs
		private const double RH_RF95_FXOSC = 32000000.0;
		private const double RH_RF95_FSTEP = RH_RF95_FXOSC / 524288.0;
		private const double RFMidBandThreshold = 525000000.0; // Search for RF_MID_BAND_THRESH GitHub LoRaNet LoRaMac-node/src/boards/sx1276-board.h
		private const int RssiAdjustmentHF = -157;
		private const int RssiAdjustmentLF = -164;

		// RegFrMsb, RegFrMid, RegFrLsb
		public const double FrequencyDefault = 434000000.0;

		// RegPAConfig
		private const Byte RegPAConfigPASelectRFO = 0b00000000;
		private const Byte RegPAConfigPASelectPABoost = 0b10000000;
		private const Byte RegPAConfigPASelectDefault = RegPAConfigPASelectRFO;
		public const bool PABoostDefault = false;

		private const byte RegPAConfigMaxPowerMin = 0b00000000;
		private const byte RegPAConfigMaxPowerMax = 0b01110000;
		public const byte RegPAConfigMaxPowerDefault = 0b01000000;

		private const byte RegPAConfigOutputPowerMin = 0b00000000;
		private const byte RegPAConfigOutputPowerMax = 0b00001111;
		public const byte RegPAConfigOutputPowerDefault = 0b00001111;

		// RegPaRamp appears to be for FSK only ?

		// RegOcp
		private const byte RegOcpOn = 0b00100000;
		private const byte RegOcpOff = 0b00000000;
		public const bool RegOcpDefault = true;

		private const byte RegOcpOcpTrimMin = 0b00000000;
		private const byte RegOcpOcpTrimMax = 0b00011111;
		public const byte RegOcpOcpTrimDefault = 0b00001011;

		// RegLna
		[Flags]
		public enum RegLnaLnaGain : byte
		{
			G1 = 0b00000001,
			G2 = 0b00000010,
			G3 = 0b00000011,
			G4 = 0b00000100,
			G5 = 0b00000101,
			G6 = 0b00000110
		}
		// Note the gain is backwards less = more
		public const RegLnaLnaGain LnaGainDefault = RegLnaLnaGain.G1;
		public const bool LnaBoostDefault = false;

		private const byte RegLnaLnaBoostLfOn = 0b00011000;
		private const byte RegLnaLnaBoostLfOff = 0b00000000;
		private const byte RegLnaLnaBoostLfDefault = RegLnaLnaBoostLfOff;

		private const byte RegLnaLnaBoostHfOn = 0b00000011;
		private const byte RegLnaLnaBoostHfOff = 0b00000000;
		private const byte RegLnaLnaBoostHfDefault = RegLnaLnaBoostHfOff;

		[Flags]
		enum RegIrqFlagsMask : byte
		{
			RxTimeoutMask = 0b10000000,
			RxDoneMask = 0b01000000,
			PayLoadCrcErrorMask = 0b00100000,
			ValidHeadrerMask = 0b00010000,
			TxDoneMask = 0b00001000,
			CadDoneMask = 0b00000100,
			FhssChangeChannelMask = 0b00000010,
			CadDetectedMask = 0b00000001,
		}

		[Flags]
		enum RegIrqFlags : byte
		{
			RxTimeout = 0b10000000,
			RxDone = 0b01000000,
			PayLoadCrcError = 0b00100000,
			ValidHeadrer = 0b00010000,
			TxDone = 0b00001000,
			CadDone = 0b00000100,
			FhssChangeChannel = 0b00000010,
			CadDetected = 0b00000001,
			ClearAll = 0b11111111,
		}

		[Flags]
		enum RegHopChannelFlags : byte
		{
			PllTimeout = 0b10000000,
			CrcOnPayload = 0b01000000,
		}

		enum RegHopChannelMask : byte
		{
			PllTimeout = 0b10000000,
			CrcOnPayload = 0b01000000,
			FhssPresentChannel = 0b01111111,
		}

		// RegModemConfig1
		public enum RegModemConfigBandwidth : byte
		{
			_7_8KHz = 0b00000000,
			_10_4KHz = 0b00010000,
			_15_6KHz = 0b00100000,
			_20_8KHz = 0b00110000,
			_31_25KHz = 0b01000000,
			_41_7KHz = 0b01010000,
			_62_5KHz = 0b01100000,
			_125KHz = 0b01110000,
			_250KHz = 0b10000000,
			_500KHz = 0b10010000
		}
		public const RegModemConfigBandwidth RegModemConfigBandwidthDefault = RegModemConfigBandwidth._125KHz;

		public enum RegModemConfigCodingRate
		{
			_4of5 = 0b00000010,
			_4of6 = 0b00000100,
			_4of7 = 0b00000110,
			_4of8 = 0b00001000,
		}
		public const RegModemConfigCodingRate RegModemConfigCodingRateDefault = RegModemConfigCodingRate._4of5;

		public enum RegModemConfigImplicitHeaderModeOn
		{
			ExplicitHeaderMode = 0b00000000,
			ImplicitHeaderMode = 0b00000001,
		}
		public const RegModemConfigImplicitHeaderModeOn RegModemConfigImplicitHeaderModeOnDefault = RegModemConfigImplicitHeaderModeOn.ExplicitHeaderMode;

		// RegModemConfig2
		public enum RegModemConfig2SpreadingFactor : byte
		{
			_64ChipsPerSymbol = 0b01100000,
			_128ChipsPerSymbol = 0b01110000,
			_256ChipsPerSymbol = 0b10000000,
			_512ChipsPerSymbol = 0b10010000,
			_1024ChipsPerSymbol = 0b10100000,
			_2048ChipsPerSymbol = 0b10110000,
			_4096ChipsPerSymbol = 0b11000000,
		}
		public const RegModemConfig2SpreadingFactor RegModemConfig2SpreadingFactorDefault = RegModemConfig2SpreadingFactor._128ChipsPerSymbol;

		private const byte RegModemConfig2TxContinuousModeOn = 0b00001000;
		private const byte RegModemConfig2TxContinuousModeOff = 0b00000000;
		public const byte RegModemConfig2TxContinuousModeDefault = RegModemConfig2TxContinuousModeOff;

		private const byte RegModemConfig2RxPayloadCrcOn = 0b00000100;
		private const byte RegModemConfig2RxPayloadCrcOff = 0b00000000;
		public const byte RegModemConfig2RxPayloadCrcDefault = RegModemConfig2RxPayloadCrcOff;

		// RegModemConfig2 for MSb RegSymbTimeoutLsb for LSB
		public const byte SymbolTimeoutDefault = 0x64;

		// RegReambleMsb & RegReambleLsb
		public const ushort PreambleLengthDefault = 0x08;

		// RegPayloadLength
		public const byte PayloadLengthDefault = 0x01;

		// RegMaxPayloadLength
		public const byte PayloadMaxLengthDefault = 0xff;

		// RegHopPeriod
		public const byte FreqHoppingPeriodDefault = 0x0;

		public const bool LowDataRateOptimizeDefault = false;

		public const bool AgcAutoOnDefault = false;

		// RegModemConfig3
		private const byte RegModemConfig3LowDataRateOptimizeOn = 0b00001000;
		private const byte RegModemConfig3LowDataRateOptimizeOff = 0b00000000;
		public const byte RegModemConfig3LowDataRateOptimizeDefault = RegModemConfig3LowDataRateOptimizeOff;

		private const byte RegModemConfig3AgcAutoOn = 0b00000100;
		private const byte RegModemConfig3AgcAutoOff = 0b00000000;
		public const byte RegModemConfig3AgcAutoDefault = RegModemConfig3AgcAutoOff;

		// RegPpmCorrection
		public const byte ppmCorrectionDefault = 0x0;

		// RegDetectOptimize
		public enum RegDetectOptimizeDectionOptimize
		{
			_SF7toSF12 = 0x03,
			_SF6 = 0x05,
		};
		public const RegDetectOptimizeDectionOptimize RegDetectOptimizeDectionOptimizeDefault = RegDetectOptimizeDectionOptimize._SF7toSF12;

		// RegInvertId
		private const byte InvertIqOn = 0b01000000;
		private const byte InvertIqOff = 0b00000000;
		public const bool InvertIqDefault = false;

		// RegDetectionThreshold
		public enum RegisterDetectionThreshold
		{
			_SF7toSF12 = 0x0A,
			_SF6 = 0x0c,
		}
		public const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold._SF7toSF12;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// RegDioMapping1 
		[Flags]
		public enum RegDioMapping1
		{
			Dio0RxDone = 0b00000000,
			Dio0TxDone = 0b01000000,
			Dio0CadDone = 0b1000000,
		}

		// The Semtech ID Relating to the Silicon revision
		private const byte RegVersionValueExpected = 0x12;

		// Hardware configuration support
		private readonly GpioPin InterruptGpioPin = null;
		private readonly GpioPin ResetGpioPin = null;
		private readonly RegisterManager RegisterManager = null;
		private readonly Object Rfm9XRegFifoLock = new object();
		private RegOpModeMode RegOpModeModeDefault = RegOpModeMode.Sleep;
		private double Frequency = FrequencyDefault;
		private bool RxDoneIgnoreIfCrcMissing = true;
		private bool RxDoneIgnoreIfCrcInvalid = true;
#if ADDRESSED_MESSAGES_PAYLOAD
		private byte[] DeviceAddress = null;
#endif

		// Constructor for RPI shields with chip select connected to CS0/CS1 and no reset pin e.g. Uputronics
		public Rfm9XDevice(ChipSelectPin chipSelectPin, int interruptPinNumber)
		{
			RegisterManager = new RegisterManager(chipSelectPin);

			// Check that SX127X chip is present
			Byte regVersionValue = RegisterManager.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			GpioController gpioController = GpioController.GetDefault();

			// Interrupt pin for RX message, TX done etc. notifications
			InterruptGpioPin = gpioController.OpenPin(interruptPinNumber);
			InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		// Constructor for RPI shields with chip select connected to CS0/CS1 e.g. Elecrow/Electronic tricks
		public Rfm9XDevice(ChipSelectPin chipSelectPin, int resetPinNumber, int interruptPinNumber)
		{
			RegisterManager = new RegisterManager(chipSelectPin);

			// Check that SX127X chip is present
			Byte regVersionValue = RegisterManager.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			GpioController gpioController = GpioController.GetDefault();

			// Setup the reset pin
			ResetGpioPin = gpioController.OpenPin(resetPinNumber);
			ResetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			ResetGpioPin.Write(GpioPinValue.High);

			// Interrupt pin for RX message, TX done etc. notifications
			InterruptGpioPin = gpioController.OpenPin(interruptPinNumber);
			InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		// Constructor for RPI shields with chip select not connected to CS0/CS1 (but needs to be configured anyway) e.g. Dragino
		public Rfm9XDevice(ChipSelectPin chipSelectPin, int chipSelectPinNumber, int resetPinNumber, int interruptPinNumber)
		{
			RegisterManager = new RegisterManager(chipSelectPin, chipSelectPinNumber);

			// Check that SX127X chip is present
			Byte regVersionValue = RegisterManager.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			GpioController gpioController = GpioController.GetDefault();

			// Setup the reset pin
			ResetGpioPin = gpioController.OpenPin(resetPinNumber);
			ResetGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			ResetGpioPin.Write(GpioPinValue.High);

			// Interrupt pin for RX message, TX done etc. notifications
			InterruptGpioPin = gpioController.OpenPin(interruptPinNumber);
			InterruptGpioPin.SetDriveMode(GpioPinDriveMode.Input);

			InterruptGpioPin.ValueChanged += InterruptGpioPin_ValueChanged;
		}

		public void RegisterDump()
		{
			RegisterManager.Dump((byte)Registers.MinValue, (byte)Registers.MaxValue);
		}

		public void SetMode(RegOpModeMode mode)
		{
			byte regOpModeValue;

			regOpModeValue = RegOpModeLongRangeModeLoRa;
			regOpModeValue |= RegOpModeAcessSharedRegLoRa;
			if (Frequency > RFMidBandThreshold)
			{
				regOpModeValue |= RegOpModeLowFrequencyModeOnHighFrequency;
			}
			else
			{
				regOpModeValue |= RegOpModeLowFrequencyModeOnLowFrequency;
			}
			regOpModeValue |= (byte)mode;
			RegisterManager.WriteByte((byte)Registers.RegOpMode, regOpModeValue);
		}

		public void Initialise( 
			double frequency = FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
			bool rxDoneignoreIfCrcMissing = true, bool rxDoneignoreIfCrcInvalid = true,
			bool paBoost = PABoostDefault, byte maxPower = RegPAConfigMaxPowerDefault, byte outputPower = RegPAConfigOutputPowerDefault, // RegPaConfig
			bool ocpOn = RegOcpDefault, byte ocpTrim = RegOcpOcpTrimDefault, // RegOcp
			RegLnaLnaGain lnaGain = LnaGainDefault, bool lnaBoost = LnaBoostDefault, // RegLna
			RegModemConfigBandwidth bandwidth = RegModemConfigBandwidthDefault, RegModemConfigCodingRate codingRate = RegModemConfigCodingRateDefault, RegModemConfigImplicitHeaderModeOn implicitHeaderModeOn = RegModemConfigImplicitHeaderModeOnDefault, //RegModemConfig1
			RegModemConfig2SpreadingFactor spreadingFactor = RegModemConfig2SpreadingFactorDefault, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
			ushort symbolTimeout = SymbolTimeoutDefault,
			ushort preambleLength = PreambleLengthDefault,
			byte payloadLength = PayloadLengthDefault,
			byte payloadMaxLength = PayloadMaxLengthDefault,
			byte freqHoppingPeriod = FreqHoppingPeriodDefault,
			bool lowDataRateOptimize = LowDataRateOptimizeDefault, bool agcAutoOn = AgcAutoOnDefault,
			byte ppmCorrection = ppmCorrectionDefault,
			RegDetectOptimizeDectionOptimize detectionOptimize = RegDetectOptimizeDectionOptimizeDefault,
			bool invertIQ = InvertIqDefault,
			RegisterDetectionThreshold detectionThreshold = RegisterDetectionThresholdDefault,
			byte syncWord = RegSyncWordDefault)
		{
			Frequency = frequency; // Store this away for RSSI adjustments
			RxDoneIgnoreIfCrcMissing = rxDoneignoreIfCrcMissing;
			RxDoneIgnoreIfCrcInvalid = rxDoneignoreIfCrcInvalid;

			// If the HopeRF module doesn't have the reset pin connected (e.g. uputroncis) not point in resetting it
			if (ResetGpioPin != null)
			{
				// Strobe Reset pin briefly to factory reset SX127X chip
				ResetGpioPin.Write(GpioPinValue.Low);
				Task.Delay(10);
				ResetGpioPin.Write(GpioPinValue.High);
				Task.Delay(10);
			}

			// Put the device into sleep mode so registers can be changed
			SetMode(RegOpModeMode.Sleep);

			// Configure RF Carrier frequency 
			if (frequency != FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / RH_RF95_FSTEP));
				RegisterManager.WriteByte((byte)Registers.RegFrMsb, bytes[2]);
				RegisterManager.WriteByte((byte)Registers.RegFrMid, bytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegFrLsb, bytes[0]);
			}

			// Set RegPaConfig if any of the associated settings are not defaults
			if ((paBoost != false) || (maxPower != RegPAConfigMaxPowerDefault) || (outputPower != RegPAConfigOutputPowerDefault))
			{
				byte regPaConfigValue = maxPower;
				if (paBoost)
				{
					regPaConfigValue |= RegPAConfigPASelectPABoost;
				}
				regPaConfigValue |= outputPower;
				RegisterManager.WriteByte((byte)Registers.RegPAConfig, regPaConfigValue);
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != true) || (ocpTrim != RegOcpOcpTrimDefault))
			{
				byte regOcpValue = ocpTrim;
				if (ocpOn)
				{
					regOcpValue |= RegOcpOn;
				}
				RegisterManager.WriteByte((byte)Registers.RegOcp, regOcpValue);
			}

			// Set RegLna if any of the settings not defaults
			if ((lnaGain != LnaGainDefault) || (lnaBoost != false))
			{
				byte regLnaValue = (byte)lnaGain;
				if (lnaBoost)
				{
					if (Frequency > RFMidBandThreshold)
					{
						regLnaValue |= RegLnaLnaBoostHfOn;
					}
					else
					{
						regLnaValue |= RegLnaLnaBoostLfOn;
					}
				}
				RegisterManager.WriteByte((byte)Registers.RegLna, regLnaValue);
			}

			// Set regModemConfig1 if any of the settings not defaults
			if ((bandwidth != RegModemConfigBandwidthDefault) || (codingRate != RegModemConfigCodingRateDefault) || (implicitHeaderModeOn != RegModemConfigImplicitHeaderModeOnDefault))
			{
				byte regModemConfig1Value = (byte)bandwidth;
				regModemConfig1Value |= (byte)codingRate;
				regModemConfig1Value |= (byte)implicitHeaderModeOn;
				RegisterManager.WriteByte((byte)Registers.RegModemConfig1, regModemConfig1Value);
			}

			// Set regModemConfig2 if any of the settings not defaults
			if ((spreadingFactor != RegModemConfig2SpreadingFactorDefault) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != SymbolTimeoutDefault))
			{
				byte RegModemConfig2Value = (byte)spreadingFactor;
				if (txContinuousMode)
				{
					RegModemConfig2Value |= RegModemConfig2TxContinuousModeOn;
				}
				if (rxPayloadCrcOn)
				{
					RegModemConfig2Value |= RegModemConfig2RxPayloadCrcOn;
				}
				// Get the MSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				RegModemConfig2Value |= symbolTimeoutBytes[1];
				RegisterManager.WriteByte((byte)Registers.RegModemConfig2, RegModemConfig2Value);
			}

			// RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
			if (symbolTimeout != SymbolTimeoutDefault)
			{
				// Get the LSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				RegisterManager.WriteByte((byte)Registers.RegSymbTimeout, symbolTimeoutBytes[0]);
			}

			// RegPreambleMsb + RegPreambleLsb
			if (preambleLength != PreambleLengthDefault)
			{
				byte[] premableBytes = BitConverter.GetBytes(preambleLength);
				RegisterManager.WriteByte((byte)Registers.RegPreambleMsb, premableBytes[1]);
				RegisterManager.WriteByte((byte)Registers.RegPreambleLsb, premableBytes[0]);
			}

			// RegPayloadLength
			if (payloadLength != PayloadLengthDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegPayloadLength, payloadLength);
			}

			// RegMaxPayloadLength
			if (payloadMaxLength != PayloadMaxLengthDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegMaxPayloadLength, payloadMaxLength);
			}

			// RegHopPeriod
			if (freqHoppingPeriod != FreqHoppingPeriodDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegHopPeriod, freqHoppingPeriod);
			}

			// RegModemConfig3
			if ((lowDataRateOptimize != false) || (agcAutoOn != false))
			{
				byte regModemConfig3Value = 0;
				if (lowDataRateOptimize)
				{
					regModemConfig3Value |= RegModemConfig3LowDataRateOptimizeOn;
				}
				if (agcAutoOn)
				{
					regModemConfig3Value |= RegModemConfig3AgcAutoOn;
				}
				RegisterManager.WriteByte((byte)Registers.RegModemConfig3, regModemConfig3Value);
			}

			// RegPpmCorrection
			if (ppmCorrection != ppmCorrectionDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegPpmCorrection, ppmCorrection);
			}

			// RegDetectOptimize
			if (detectionOptimize != RegDetectOptimizeDectionOptimizeDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegDetectOptimize, (byte)detectionOptimize);
			}

			// RegInvertIQ
			if (invertIQ != false)
			{
				RegisterManager.WriteByte((byte)Registers.RegInvertIQ, (byte)InvertIqOn);
			}

			// RegDetectionThreshold
			if (detectionThreshold != RegisterDetectionThresholdDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegDetectionThreshold, (byte)detectionThreshold);
			}

			// RegSyncWordDefault 
			if (syncWord != RegSyncWordDefault)
			{
				RegisterManager.WriteByte((byte)Registers.RegSyncWord, syncWord);
			}

			// Configure RegOpMode before returning
			SetMode( RegOpModeMode.Sleep);
		}



		private void ProcessTxDone(byte IrqFlags)
		{
			Debug.Assert(IrqFlags != 0);
			SetMode(RegOpModeModeDefault);

			OnDataTransmitedEventArgs transmitArgs = new OnDataTransmitedEventArgs();

			OnTransmit?.Invoke(this, transmitArgs);
		}

		private void ProcessRxDone(byte IrqFlags)
		{
			byte[] payloadBytes;
			Debug.Assert(IrqFlags != 0);

			// Check to see if payload has CRC 
			if (RxDoneIgnoreIfCrcMissing)
			{
				byte regHopChannel = this.RegisterManager.ReadByte((byte)Registers.RegHopChannel);
				if ((regHopChannel & (byte)RegHopChannelMask.CrcOnPayload) != (byte)RegHopChannelFlags.CrcOnPayload)
				{
					return;
				}
			}

			// Check to see if payload CRC is valid
			if (RxDoneIgnoreIfCrcInvalid)
			{
				if ((IrqFlags & (byte)RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)RegIrqFlagsMask.PayLoadCrcErrorMask)
				{
					return;
				}
			}

			// Extract the message from the RFM9X fifo, try and keep lock in place for the minimum possible time
			lock (Rfm9XRegFifoLock)
			{
				byte currentFifoAddress = this.RegisterManager.ReadByte((byte)Registers.RegFifoRxCurrent);

				this.RegisterManager.WriteByte((byte)Registers.RegFifoAddrPtr, currentFifoAddress);

				byte numberOfBytes = this.RegisterManager.ReadByte((byte)Registers.RegRxNbBytes);

				// Allocate buffer for message
				payloadBytes = new byte[numberOfBytes];

				for (int i = 0; i < numberOfBytes; i++)
				{
					payloadBytes[i] = this.RegisterManager.ReadByte((byte)Registers.RegFifo);
				}
			}

#if ADDRESSED_MESSAGES_PAYLOAD
			//check that message is long enough to contain header 
			if (payloadBytes.Length < (AddressHeaderLength + AddressLengthMinimum + AddressLengthMinimum))
			{
				return;
			}

			int toAddressLength = (payloadBytes[0] >> 4) & 0xf;
			int fromAddressLength = payloadBytes[0] & 0xf;
			int messageLength = payloadBytes.Length - (AddressHeaderLength + toAddressLength + fromAddressLength);

			// Make sure the to and local addresses are the same length
			if (toAddressLength != DeviceAddress.Length)
			{
				return;
			}

			// Compare the bytes of the addresses
			for (int index = 0; index < DeviceAddress.Length; index++)
			{
				if (DeviceAddress[index] != payloadBytes[index + AddressHeaderLength])
				{
					return;
				}
			}
#endif

			// Get the RSSI HF vs. LF port adjustment section 5.5.5 RSSI and SNR in LoRa Mode
			float packetSnr = this.RegisterManager.ReadByte((byte)Registers.RegPktSnrValue) * 0.25f;

			int rssi = this.RegisterManager.ReadByte((byte)Registers.RegRssiValue);
			if (Frequency > RFMidBandThreshold)
			{
				rssi = RssiAdjustmentHF + rssi;
			}
			else
			{
				rssi = RssiAdjustmentLF + rssi;
			}

			int packetRssi = this.RegisterManager.ReadByte((byte)Registers.RegPktRssiValue);
			if (Frequency > RFMidBandThreshold)
			{
				packetRssi = RssiAdjustmentHF + packetRssi;
			}
			else
			{
				packetRssi = RssiAdjustmentLF + packetRssi;
			}

#if ADDRESSED_MESSAGES_PAYLOAD
			byte[] address = new byte[fromAddressLength];
			Array.Copy(payloadBytes, AddressHeaderLength + toAddressLength, address, 0, fromAddressLength);
			byte[] messageBytes = new byte[messageLength];
			Array.Copy(payloadBytes, AddressHeaderLength + toAddressLength + fromAddressLength, messageBytes, 0, messageLength);
#endif

			OnDataReceivedEventArgs receiveArgs = new OnDataReceivedEventArgs
			{
#if ADDRESSED_MESSAGES_PAYLOAD
				Address = address,
#endif
				PacketSnr = packetSnr,
				Rssi = rssi,
				PacketRssi = packetRssi,
#if ADDRESSED_MESSAGES_PAYLOAD
				Data = messageBytes,
#else
				Data = payloadBytes,
#endif
			};

			OnReceive?.Invoke(this, receiveArgs);
		}

		private void InterruptGpioPin_ValueChanged(GpioPin sender, GpioPinValueChangedEventArgs args)
		{
			if (args.Edge != GpioPinEdge.RisingEdge)
			{
				return;
			}

			// Read RegIrqFlags to see what caused the interrupt
			Byte IrqFlags = this.RegisterManager.ReadByte((byte)Registers.RegIrqFlags);

			// Check RxDone for inbound message
			if ((IrqFlags & (byte)RegIrqFlagsMask.RxDoneMask) == (byte)RegIrqFlags.RxDone)
			{
				ProcessRxDone(IrqFlags);
			}

			// Check TxDone for outbound message
			if ((IrqFlags & (byte)RegIrqFlagsMask.TxDoneMask) == (byte)RegIrqFlags.TxDone)
			{
				ProcessTxDone(IrqFlags);
			}

			this.RegisterManager.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio0RxDone);
			this.RegisterManager.WriteByte((byte)Registers.RegIrqFlags, (byte)RegIrqFlags.ClearAll);
		}

#if ADDRESSED_MESSAGES_PAYLOAD
		public void Send(byte[] addressBytes, byte[] messageBytes)
		{
			Debug.Assert(addressBytes != null);
			Debug.Assert(addressBytes.Length > AddressLengthMinimum);
			Debug.Assert(addressBytes.Length < AddressLengthMaximum);
			Debug.Assert(messageBytes != null);
			Debug.Assert(messageBytes.Length > 0);

			// construct payload from lengths and addresses
			byte[] payLoadBytes = new byte[AddressHeaderLength + addressBytes.Length + DeviceAddress.Length + messageBytes.Length];

			byte toAddressLength = (byte)(addressBytes.Length << 4);
			byte fromAddressLength = (byte)DeviceAddress.Length;
			payLoadBytes[0] = (byte)(toAddressLength | fromAddressLength);

			// copy across the to & from addresses
			Array.Copy(addressBytes, 0, payLoadBytes, 1, addressBytes.Length);
			Array.Copy(DeviceAddress, 0, payLoadBytes, 1 + addressBytes.Length, DeviceAddress.Length);

			// copy across the payload
			Array.Copy(messageBytes, 0, payLoadBytes, 1 + addressBytes.Length + DeviceAddress.Length, messageBytes.Length);

			Send(payLoadBytes);
		}

		public void Receive( byte[] address )
		{
			Debug.Assert(address != null);
			Debug.Assert(address.Length >= AddressLengthMinimum);
			Debug.Assert(address.Length <= AddressLengthMaximum);
			DeviceAddress = address;

			RegOpModeModeDefault = RegOpModeMode.ReceiveContinuous ;
			SetMode(RegOpModeMode.ReceiveContinuous);
		}
#else
		public void Receive()
		{
			RegOpModeModeDefault = RegOpModeMode.ReceiveContinuous ;
			SetMode(RegOpModeMode.ReceiveContinuous);
		}
#endif

#if ADDRESSED_MESSAGES_PAYLOAD
		private void Send(byte[] messageBytes)
#else
		public void Send(byte[] messageBytes)
#endif
		{
			Debug.Assert(messageBytes != null);
			Debug.Assert(messageBytes.Length > 0);

			lock (Rfm9XRegFifoLock)
			{
				this.RegisterManager.WriteByte((byte)Registers.RegFifoTxBaseAddr, 0x0);

				// Set the Register Fifo address pointer
				this.RegisterManager.WriteByte((byte)Registers.RegFifoAddrPtr, 0x0);

				foreach (byte b in messageBytes)
				{
					this.RegisterManager.WriteByte((byte)Registers.RegFifo, b);
				}

				// Set the length of the message in the fifo
				this.RegisterManager.WriteByte((byte)Registers.RegPayloadLength, (byte)messageBytes.Length);
			}

			this.RegisterManager.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio0TxDone);
			SetMode(RegOpModeMode.Transmit);
		}
	}
}