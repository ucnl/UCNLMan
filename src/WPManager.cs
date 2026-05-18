using UCNLNav;
using UCNLPhysics;
using UCNLSalinity;

public class WPManager : IDisposable
{
    #region Constants
    private const double SALINITY_CHANGE_THRESHOLD = 0.1;
    private const double GRAVITY_CHANGE_THRESHOLD = 0.0001;
    private const double AUTO_SALINITY_RANGE_THRESHOLD = 100000; // 100 km in meters
    private const double DEFAULT_LATITUDE = 48;
    private const double DEFAULT_LONGITUDE = 44;
    #endregion

    #region Fields
    private readonly WWSalinityProvider _salinityProvider;

    private double _soundSpeed = PHX.PHX_FWTR_SOUND_SPEED_MPS;
    private double _salinity = PHX.PHX_FWTR_SALINITY_PSU;
    private double _gravityAcceleration = PHX.PHX_GRAVITY_ACC_MPS2;
    private double _latitude = DEFAULT_LATITUDE;
    private double _longitude = DEFAULT_LONGITUDE;
    private double temperature = 0;
    private double pressure = PHX.PHX_ATM_PRESSURE_MBAR;
    private double atmospheric_pressure = PHX.PHX_ATM_PRESSURE_MBAR;
    #endregion

    #region Properties
    public bool IsAutoSalinity { get; set; }
    public bool IsAutoSoundSpeed { get; set; }
    public bool IsAutoGravity { get; set; }

    public double SalinityLatPoint { get; private set; } = DEFAULT_LATITUDE;
    public double SalinityLonPoint { get; private set; } = DEFAULT_LONGITUDE;

    private double salinity
    {
        get => _salinity;
        set
        {
            if (Math.Abs(_salinity - value) <= SALINITY_CHANGE_THRESHOLD)
                return;

            _salinity = value;
            SalinityChanged?.Invoke(this, EventArgs.Empty);

            if (IsAutoSoundSpeed)
                UpdateSoundSpeed();
        }
    }

    private double soundSpeed
    {
        get => _soundSpeed;
        set
        {
            if (Math.Abs(_soundSpeed - value) <= SALINITY_CHANGE_THRESHOLD)
                return;

            _soundSpeed = value;
            SoundSpeedChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    private double gravityAcceleration
    {
        get => _gravityAcceleration;
        set
        {
            if (Math.Abs(_gravityAcceleration - value) <= GRAVITY_CHANGE_THRESHOLD)
                return;

            _gravityAcceleration = value;
            GravityAccelerationChanged?.Invoke(this, EventArgs.Empty);
        }
    }

    public double Temperature
    {
        get => temperature;
        set
        {
            temperature = value;
            if (IsAutoSoundSpeed)
                UpdateSoundSpeed();
        }
    }

    public double Pressure
    {
        get => pressure;
        set
        {
            pressure = value;
            if (IsAutoSoundSpeed)
                UpdateSoundSpeed();
        }
    }

    public double AtmosphericPressure
    {
        get => atmospheric_pressure;
        set
        {
            atmospheric_pressure = value;
            if (IsAutoSoundSpeed)
                UpdateSoundSpeed();
        }
    }

    public double SoundSpeed
    {
        get => _soundSpeed;
        set
        {
            ValidateSoundSpeed(value);
            soundSpeed = value;
            IsAutoSoundSpeed = false;
        }
    }

    public double Salinity
    {
        get => _salinity;
        set
        {
            ValidateSalinity(value);
            IsAutoSalinity = false;
            salinity = value;
        }
    }

    public double GravityAcceleration
    {
        get => _gravityAcceleration;
        set
        {
            ValidateGravity(value);
            gravityAcceleration = value;
            IsAutoGravity = false;
        }
    }

    public double Latitude_Deg
    {
        get => _latitude;
        private set
        {
            ValidateLatitude(value);
            _latitude = value;
            if (IsAutoGravity)
                gravityAcceleration = PHX.Gravity_constant_wgs84_calc(_latitude);
        }
    }

    public double Longitude_Deg
    {
        get => _longitude;
        private set
        {
            ValidateLongitude(value);
            _longitude = value;
        }
    }
    #endregion

    #region Constructor


    public WPManager()
    {
        _salinityProvider = new WWSalinityProvider();
    }

    public WPManager(string salinityDataFile)
    {
        _salinityProvider = new WWSalinityProvider(salinityDataFile);
    }
    #endregion

    #region Validation
    private static void ValidateLatitude(double latitude)
    {
        if (latitude < -90 || latitude > 90)
            throw new ArgumentOutOfRangeException(nameof(latitude), "Latitude must be between -90 and 90 degrees");
    }

    private static void ValidateLongitude(double longitude)
    {
        if (longitude < -180 || longitude > 180)
            throw new ArgumentOutOfRangeException(nameof(longitude), "Longitude must be between -180 and 180 degrees");
    }

    private static void ValidateSalinity(double salinity)
    {
        if (salinity < PHX.PHX_SALINITY_PSU_MIN || salinity > PHX.PHX_SALINITY_PSU_MAX)
            throw new ArgumentOutOfRangeException(nameof(salinity),
                $"Salinity must be between {PHX.PHX_SALINITY_PSU_MIN} and {PHX.PHX_SALINITY_PSU_MAX} PSU");
    }

    private static void ValidateSoundSpeed(double speed)
    {
        if (speed < PHX.PHX_FWTR_SOUND_SPEED_MPS_MIN || speed > PHX.PHX_FWTR_SOUND_SPEED_MPS_MAX)
            throw new ArgumentOutOfRangeException(nameof(speed),
                $"Sound speed must be between {PHX.PHX_FWTR_SOUND_SPEED_MPS_MIN} and {PHX.PHX_FWTR_SOUND_SPEED_MPS_MAX} m/s");
    }

    private static void ValidateGravity(double gravity)
    {
        if (gravity < PHX.PHX_GRAVITY_ACC_MPS2_MIN || gravity > PHX.PHX_GRAVITY_ACC_MPS2_MAX)
            throw new ArgumentOutOfRangeException(nameof(gravity),
                $"Gravity must be between {PHX.PHX_GRAVITY_ACC_MPS2_MIN} and {PHX.PHX_GRAVITY_ACC_MPS2_MAX} m/s²");
    }
    #endregion

    #region Methods
    private void UpdateSoundSpeed()
    {
        if (!IsAutoSoundSpeed) return;

        var newSpeed = PHX.Speed_of_sound_UNESCO_calc(temperature, pressure, salinity);
        soundSpeed = newSpeed;
    }

    public void UpdateLocation(double lat_deg, double lon_deg)
    {
        Latitude_Deg = lat_deg;
        Longitude_Deg = lon_deg;

        Algorithms.VincentyInverse(
            Algorithms.Deg2Rad(lat_deg),
            Algorithms.Deg2Rad(lon_deg),
            Algorithms.Deg2Rad(SalinityLatPoint),
            Algorithms.Deg2Rad(SalinityLonPoint),
            Algorithms.WGS84Ellipsoid,
            Algorithms.VNC_DEF_EPSILON,
            Algorithms.VNC_DEF_IT_LIMIT,
            out double range,
            out _,
            out _,
            out _);

        if (range > AUTO_SALINITY_RANGE_THRESHOLD && IsAutoSalinity)
        {
            UpdateSalinityFromProvider();
        }
    }

    private void UpdateSalinityFromProvider()
    {
        try
        {
            var newSalinity = _salinityProvider.GetNearestSalinity(
                _latitude,
                _longitude,
                out double latPoint,
                out double lonPoint);

            SalinityLatPoint = latPoint;
            SalinityLonPoint = lonPoint;

            salinity = newSalinity;
        }
        catch (Exception ex)
        {
            
        }
    }
    #endregion

    #region Events
    public event EventHandler? SoundSpeedChanged;
    public event EventHandler? SalinityChanged;
    public event EventHandler? GravityAccelerationChanged;
    #endregion

    #region IDisposable
    public void Dispose()
    {
        (_salinityProvider as IDisposable)?.Dispose();
    }
    #endregion
}
