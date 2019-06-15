# Estimated local position, velocity, acceleration in NED frame
#uint64 timestamp_estimate

# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

# Velocity in NED frame
float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)

# Acceleration in NED frame
float32 ax        		# North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay        		# East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az        		# Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

# Log dt
float32 dt