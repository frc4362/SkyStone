package com.gemsrobotics.lib;

public final class DeltaTime {
	private long m_lastTime;
	private boolean m_isLaunched;

	public DeltaTime() {
		m_isLaunched = false;
	}

	public boolean isLaunched(){
		return m_isLaunched;
	}

	public void launch() {
		m_lastTime = System.currentTimeMillis();
		m_isLaunched = true;
	}

	public double update() {
		final long now = System.currentTimeMillis();
		final long delta = now - m_lastTime;
		return ((double) delta) / 1000.0;
	}
}
