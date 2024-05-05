import { TypedEventTarget } from "typescript-event-target";

export type FakeRw0003Input = {
  torque: number;
};

const decodeInput = (data: DataView): FakeRw0003Input => {
  if (data.byteLength !== 4) {
    throw new Error("Invalid input report");
  }
  const torque = data.getFloat32(0, true);
  return { torque };
};

export class FakeRw0003InputEvent extends Event {
  constructor(
    public readonly input: FakeRw0003Input,
    init?: EventInit
  ) {
    super("input", init);
  }
}

export type FakeRw0003Output = {
  temperature: number;
  speed: number;
};

const encodeOutput = (output: FakeRw0003Output): ArrayBuffer => {
  const data = new DataView(new ArrayBuffer(8));
  data.setFloat32(0, output.temperature, true);
  data.setFloat32(4, output.speed, true);
  return data.buffer;
};

type EventMap = {
  input: FakeRw0003InputEvent;
};

export class FakeRw0003 extends TypedEventTarget<EventMap> {
  private hid: HIDDevice | null = null;

  isConnected() {
    return this.hid !== null && this.hid.opened;
  }

  async connect() {
    if (this.hid !== null) {
      throw new Error("Already connected");
    }
    try {
      const devices = await navigator.hid.requestDevice({
        filters: [{ vendorId: 0x6666, productId: 0x6910 }],
      });
      if (devices.length === 0) {
        throw new Error("No device found");
      }
      this.hid = devices[0];
      this.hid.addEventListener("inputreport", this.handleInputReport);
      this.hid.addEventListener("disconnect", this.handleDisconnect);
      await this.hid.open();
    } catch (err) {
      this.disconnect();
    }
  }

  handleInputReport = (hidEvent: HIDInputReportEvent) => {
    const input = decodeInput(hidEvent.data);
    const event = new FakeRw0003InputEvent(input);
    this.dispatchTypedEvent("input", event);
  };

  handleDisconnect = () => {
    this.disconnect();
  };

  disconnect() {
    this.hid?.removeEventListener("inputreport", this.handleInputReport);
    this.hid?.close();
    this.hid = null;
  }

  sendOutput(output: FakeRw0003Output) {
    if (this.hid === null) {
      throw new Error("Not connected");
    }
    this.hid.sendReport(0, encodeOutput(output));
  }
}
