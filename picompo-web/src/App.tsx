import React, { useCallback, useEffect, useState } from "react";
import { Button, H2, Intent, Navbar, ProgressBar, Slider } from "@blueprintjs/core";
import {
  FakeRw0003,
  FakeRw0003Input,
  FakeRw0003InputEvent,
} from "./fakerw0003";

const useFakeRw0003 = () => {
  const [fakeRw0003] = useState(() => {
    return new FakeRw0003();
  });
  const [input, setInput] = useState<FakeRw0003Input>({
    torque: 0,
  });
  const handleInput = useCallback((event: FakeRw0003InputEvent) => {
    setInput(event.input);
  }, []);

  useEffect(() => {
    fakeRw0003.addEventListener("input", handleInput);
    return () => {
      fakeRw0003.removeEventListener("input", handleInput);
    };
  }, [fakeRw0003, handleInput]);
  useEffect(() => {
    return () => {
      fakeRw0003.disconnect();
    };
  }, [fakeRw0003]);

  return {
    fakeRw0003,
    input,
  };
};

const MAX_TORQUE = 0.005;
const MAX_SPEED = 1000;
const MAX_TEMPERATURE = 100;
export const App: React.FC = () => {
  const { fakeRw0003, input } = useFakeRw0003();
  const torquePercentage =
    (Math.min(input.torque, MAX_TORQUE) / MAX_TORQUE) * 100;
  const [speed, setSpeed] = useState(0);
  const [temperature, setTemperature] = useState(0);

  const handleSpeedChange = useCallback(
    (speed: number) => {
      setSpeed(speed);
      fakeRw0003.sendOutput({ speed, temperature });
    },
    [fakeRw0003, temperature]
  );
  const handleTemperatureChange = useCallback(
    (temperature: number) => {
      setTemperature(temperature);
      fakeRw0003.sendOutput({ speed, temperature });
    },
    [fakeRw0003, speed]
  );

  const handleConnectClick = useCallback(() => {
    fakeRw0003.connect();
  }, [fakeRw0003]);

  return (
    <>
      <Navbar>
        <Navbar.Group>
          <Navbar.Heading>picompo</Navbar.Heading>
        </Navbar.Group>
      </Navbar>
      <div className="mx-auto my-6 max-w-[960px]">
        <H2>RW0.0003</H2>
        <div className="relative px-16">
          <div className="flex flex-row items-center my-2 space-x-4">
            <div className="w-16">Torque</div>
            <div className="w-16 text-right font-mono">{input.torque}</div>
            <ProgressBar
              animate={false}
              value={torquePercentage}
              className="flex-1"
            />
          </div>
          <div className="flex flex-row items-center my-2 space-x-4">
            <div className="w-16">Speed</div>
            <div className="w-16 text-right font-mono">{speed}</div>
            <Slider
              min={0}
              max={MAX_SPEED}
              value={speed}
              onChange={handleSpeedChange}
              labelStepSize={100}
              className="flex-1"
            />
          </div>
          <div className="flex flex-row items-center my-2 space-x-4">
            <div className="w-16">Temperature</div>
            <div className="w-16 text-right font-mono">{temperature}</div>
            <Slider
              min={0}
              max={MAX_TEMPERATURE}
              value={temperature}
              onChange={handleTemperatureChange}
              labelStepSize={10}
              className="flex-1"
            />
          </div>
          {!fakeRw0003.isConnected() && (
            <div className="absolute inset-0 grid place-content-center backdrop-grayscale backdrop-blur-sm">
              <Button intent={Intent.PRIMARY} onClick={handleConnectClick}>Connect</Button>
            </div>
          )}
        </div>
      </div>
    </>
  );
};
