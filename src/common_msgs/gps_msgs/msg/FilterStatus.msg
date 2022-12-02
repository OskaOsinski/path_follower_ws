uint8 FIX_NO_GNSS_FIX=0
uint8 FIX_2D_GNSS_FIX=1
uint8 FIX_3D_GNSS_FIX=2
uint8 FIX_SBAS_GNSS_FIX=3
uint8 FIX_DIFFERENTIAL_GNSS_FIX=4
uint8 FIX_OMNISTAR_GNSS_FIX=5
uint8 FIX_RTK_FLOAT_GNSS_FIX=6
uint8 FIX_FTK_FIXED_GNSS_FIX=7

uint16 raw_status

bool OrientationFilterInitialized
bool NavigationFilterInitialized
bool HeadingInitialized
bool UTCTimeInitialized

uint8 GNSSFixStatus

bool Event1Occured
bool Event2Occured

bool InternalGNSSEnabled
bool DualAntennaHeadingActive
bool VelocityHeadingEnabled
bool AtmosphericAltitudeEnabled

bool ExternalPositionActive
bool ExternalVelocityActive
bool ExternalHeadingActive
