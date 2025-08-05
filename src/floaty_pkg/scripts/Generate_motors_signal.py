import numpy as np
import scipy
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft

def generate_signal_with_frequencies(freqs, phases, amplitudes, time_step, period, shift):

    time_array = np.arange(0, period,time_step)
    signal = []
    for t in time_array:
        sum = 0
        for i, freq in enumerate(freqs):
            if freq == 0:
                sum+= amplitudes[i]
            else:
                sum+= 2*np.cos(2*np.pi*freq*t + phases[i])*amplitudes[i]
        signal.append(sum)

    return time_array, signal


def generate_signal_in_frequency_domain(freqs, phases, amplitudes, freq_resolution, max_freq, shift_in_freqs):

    freqs_array = [0 for i in range(int(2*max_freq/freq_resolution))]
    for i, f in enumerate(freqs):
        freq_pos = int(((f+1e-4)/freq_resolution)+1e-4)
        # res = amplitudes[i]*(np.cos(phases[i]+shift_in_freqs)+ np.sin(phases[i]+shift_in_freqs)*1j)
        if f ==0:
            res = amplitudes[i]*np.exp((phases[i]+shift_in_freqs)*1j)
            freqs_array[freq_pos]=res   
        else:
            res = amplitudes[i]*np.exp((phases[i]+shift_in_freqs)*1j)
            freqs_array[freq_pos]=res
            freqs_array[-freq_pos]=np.conjugate(res)

    return freqs_array


def write_signal_to_file(file, signal_num, signal, table_name="lookup_table"):
    file.write("Motor number {num}.\n".format(num=signal_num))
    file.write("const int {table_name}[{length}] = ".format(table_name=table_name, length=signal.size))
    file.write("{")
    file.write(str(int(signal[0])))
    for sig in signal[1:]:
        file.write(", "+str(int(sig)))
    file.write("};")
    file.write("\n \n")


scale = 1/3
max_motor_range_deg = 120
max_cf_range = 65535
deg_to_cf_range = max_cf_range/max_motor_range_deg
shifts_for_motors_deg = [95, 25, 95, 25]
amplitude_scal = scale*deg_to_cf_range

shifts_for_motors = [shift_*deg_to_cf_range for shift_ in shifts_for_motors_deg]

# freqs = np.linspace(0, 50, 501)
# freqs = np.array([f/10 for f in range(500)])
freqs = np.array([f/10 for f in range(201)])
used_freqs = freqs
# phases = [0.1 for i, f in enumerate(freqs)]
# phases = [np.pi/10*(i%20)  for i, f in enumerate(freqs)]

phases = np.random.random(freqs.size)*np.pi*2
phases2 = np.array([(ang+np.pi)%(2*np.pi) for ang in phases])

# amplitudes = [amplitude_scal for i, f in enumerate(freqs)]
amplitudes = [amplitude_scal if f in used_freqs else 0 for i, f in enumerate(freqs)]
amplitudes2 = [amplitude_scal if f in used_freqs else 0 for i, f in enumerate(freqs)]
# amplitudes = [1 if f in used_freqs else 0 for i, f in enumerate(freqs)]
amplitudes[0] = shifts_for_motors[0]
# amplitudes2[0] = shifts_for_motors[1]   # For motors 2 & 4
period = 10
time_step = 0.01
freq_resolution = 1/period
max_freq = 1/(time_step*2)

phases[0]=0
phases2[0]=0

freq_domain = generate_signal_in_frequency_domain(freqs, phases, amplitudes, freq_resolution, max_freq, 0)
# freq_domain2 = generate_signal_in_frequency_domain(freqs, phases2, amplitudes2, freq_resolution, max_freq, 0)
DFT_matrix = [1, 1, 1, 1]

# time_domain = np.real(scipy.fft.ifft(freq_domain, norm="forward"))
time_domain = np.real(ifft(freq_domain, norm="forward"))
time_domain2 = shifts_for_motors[1] - (time_domain-amplitudes[0])
# time_array = np.arange(0, period,time_step)

# time_domain2_freq = np.real(ifft(freq_domain2, norm="forward"))

# test_diff = time_domain2_freq-time_domain2


t, signal = generate_signal_with_frequencies(freqs, phases, amplitudes, time_step, period, shifts_for_motors[0])

fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(4.8, 7))

signal_freq = np.real(time_domain)
signal_dif = signal-signal_freq
ax0.plot(t, signal)
ax0.set_title("Signal in time domain")
ax1.plot(t, signal_dif)
ax1.set_title("Signal error in time domain")
plt.show()

tpCount = len(signal)
values = np.arange(int(tpCount/2))
timePeriod  = tpCount*time_step
frequencies = values/timePeriod


values2 = np.arange(int(tpCount))
frequencies2 = values2/timePeriod

fft_res = np.fft.fft(signal)/len(signal)
# fft_res = fft[range(int(len(signal)/2))]

fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(4.8, 7))

amps = np.abs(fft_res)
angs = [np.angle(f) if (abs(amps[i])>0.001) else 0 for (i,f) in enumerate(fft_res)]


freqs_list = freqs.tolist()
phases_list = phases.tolist()
phases2_list = phases2.tolist()


# file = open("flaps_signal_data.txt", "w")
# file.write("frequencies: \n")
# # file.write(str(freqs))
# file.write(str(freqs_list))
# file.write("\n \n")
# file.write("amplitudes: \n")
# file.write(str(amplitudes))
# file.write("\n \n")
# file.write("phases: \n")
# # file.write(str(phases))
# file.write(str(phases_list))
# file.write("\n \n")
# file.write("phases2 : \n")
# # file.write(str(phases2))
# file.write(str(phases2_list))
# file.write("\n \n")
# file.write("Shifts for motors: \n")
# file.write(str(shifts_for_motors))
# file.write("\n \n")
# file.write("period: "+str(period)+"\n")
# file.write("time_step: "+str(time_step)+"\n")
# file.write("\n \n")


# file.write("signals: \n\n")

# write_signal_to_file(file, 1, time_domain)

# write_signal_to_file(file, 2, time_domain2, table_name="lookup_table_2")

# # file.write("const int lookup_table[{length}] = ".format(length=time_domain.size))
# # file.write("{")
# # file.write(str(int(time_domain[0])))
# # for sig in time_domain[1:]:
# #     file.write(", "+str(int(sig)))
# # file.write("};")
# # file.write("\n \n")

# # file.write("const int lookup_table_2[{length}] = ".format(length=time_domain.size))
# # file.write("{")
# # file.write(str(int(time_domain2[0])))
# # for sig in time_domain2[1:]:
# #     file.write(", "+str(int(sig)))
# # file.write("};")
# # file.write("\n \n")

# file.close()


angs = [180/np.pi*ang for ang in angs]

ax0.plot(frequencies2, 20*np.log(np.abs(fft_res)))
ax1.plot(frequencies2, angs)

ax0.set_title("Signal in frequency domain")
ax0.set_ylabel("Amplitude dB")
ax0.set_xlabel("Frequency Hz")
ax1.set_ylabel("Phase")
ax1.set_xlabel("Frequency Hz")
ax1.set_title("Signal phases")

plt.show()



fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(4.8, 7))

amps = np.abs(freq_domain)
angs = [np.angle(f) if (abs(amps[i])>0.01) else 0 for (i,f) in enumerate(fft_res)]

ax0.plot(frequencies2, 20*np.log(amps))
ax1.plot(frequencies2, angs)

ax0.set_title("Signal in frequency domain")
ax0.set_ylabel("Amplitude dB")
ax0.set_xlabel("Frequency Hz")
ax1.set_ylabel("Phase")
ax1.set_xlabel("Frequency Hz")
ax1.set_title("Signal phases")

plt.show()

print("end")
