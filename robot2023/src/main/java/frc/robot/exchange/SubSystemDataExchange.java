package frc.robot.exchange;

public class SubSystemDataExchange {

    private boolean _isSpeedLimited;
    
    public SubSystemDataExchange() {
        _isSpeedLimited = false;
    }

    public void setSpeedLimited(boolean isSpeedLimited) {
        _isSpeedLimited = isSpeedLimited;
    }

    public boolean getSpeedLimited() {
        return _isSpeedLimited;
    }

}