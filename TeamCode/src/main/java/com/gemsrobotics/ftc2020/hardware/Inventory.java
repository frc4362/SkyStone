package com.gemsrobotics.ftc2020.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Inventory {
	private final DigitalChannel m_switchLeft, m_switchRight;

	public Inventory(final HardwareMap hardwareMap) {
		m_switchLeft = hardwareMap.get(DigitalChannel.class, "LeftSwitch");
		m_switchLeft.setMode(DigitalChannel.Mode.OUTPUT);
		m_switchRight = hardwareMap.get(DigitalChannel.class, "RightSwitch");
		m_switchLeft.setMode(DigitalChannel.Mode.OUTPUT);
	}

	public boolean isCubeTiltedLeft() {
		return m_switchLeft.getState();
	}

	public boolean isCubeTiltedRight() {
		return m_switchRight.getState();
	}
}
