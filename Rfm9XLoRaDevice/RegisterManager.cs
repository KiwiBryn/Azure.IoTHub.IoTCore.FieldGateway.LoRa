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
	using System.Runtime.InteropServices.WindowsRuntime;
	using Windows.Devices.Gpio;
	using Windows.Devices.Spi;

	public enum ChipSelectPin
	{
		CS0 = 0,
		CS1 = 1
	}

	public sealed class RegisterManager
	{
		private GpioPin ChipSelectGpioPin = null;
		private SpiDevice Device = null;
		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		public RegisterManager(ChipSelectPin chipSelectPin)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings((int)chipSelectPin)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			Device = spiController.GetDevice(settings);
		}

		public RegisterManager(ChipSelectPin chipSelectPinDummy, int chipSelectPinNumber)
		{
			SpiController spiController = SpiController.GetDefaultAsync().AsTask().GetAwaiter().GetResult();
			var settings = new SpiConnectionSettings((int)chipSelectPinDummy)
			{
				ClockFrequency = 500000,
				Mode = SpiMode.Mode0,
			};

			// Chip select pin configuration
			GpioController gpioController = GpioController.GetDefault();
			ChipSelectGpioPin = gpioController.OpenPin(chipSelectPinNumber);
			ChipSelectGpioPin.SetDriveMode(GpioPinDriveMode.Output);
			ChipSelectGpioPin.Write(GpioPinValue.High);

			Device = spiController.GetDevice(settings);
		}

		public Byte ReadByte(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[1];
			Debug.Assert(Device != null);

			if (ChipSelectGpioPin!= null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.TransferSequential(writeBuffer, readBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}
			return readBuffer[0];
		}

		public ushort ReadWord(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[2];
			Debug.Assert(Device != null);

			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.TransferSequential(writeBuffer, readBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}

			return (ushort)(readBuffer[1] + (readBuffer[0] << 8));
		}

		public byte[] Read(byte address, int length)
		{
			byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask };
			byte[] readBuffer = new byte[length];
			Debug.Assert(Device != null);

			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.TransferSequential(writeBuffer, readBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}
			return readBuffer;
		}

		public void WriteByte(byte address, byte value)
		{
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
			Debug.Assert(Device != null);

			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.Write(writeBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}
		}

		public void WriteWord(byte address, ushort value)
		{
			byte[] valueBytes = BitConverter.GetBytes(value);
			byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
			Debug.Assert(Device != null);

			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.Write(writeBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}
		}

		public void Write(byte address, [ReadOnlyArray()] byte[] bytes)
		{
			byte[] writeBuffer = new byte[1 + bytes.Length];
			Debug.Assert(Device != null);

			Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
			writeBuffer[0] = address |= RegisterAddressWriteMask;

			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.Low);
			}
			Device.Write(writeBuffer);
			if (ChipSelectGpioPin != null)
			{
				ChipSelectGpioPin.Write(GpioPinValue.High);
			}
		}

		public void Dump(byte start, byte finish)
		{
			Debug.Assert(Device != null);

			Debug.WriteLine("Register dump");

			for (byte registerIndex = start; registerIndex <= finish; registerIndex++)
			{
				byte registerValue = this.ReadByte(registerIndex);

				Debug.WriteLine("Register 0x{0:x2} - Value 0X{1:x2} - Bits {2}", registerIndex, registerValue, Convert.ToString(registerValue, 2).PadLeft(8, '0'));
			}
		}
	}
}
