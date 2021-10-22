uint64 timestamp			# time since system start (microseconds)

#EKF
float32 xk_km1
float32 hk
float32 pk_km1
float32 ykmodel
float32 yk
float32 sk
#EKF

bool ekf_flag

#Complimentary
float32 va_wind
float32 phi
float32 alpha
#Complimentary

#Final estimate
float32 va_est
#Final estimate
