clc;
gain1=2;    %gain for low pass
gain2=1;     %gain for bandpass
gain3=0;     %gain for highpass
final_array = gain1*low_pass1+ gain2*band_pass1 + gain3*high_pass1;
Fs = 44100;  % Sampling frequency 44.1 kHz

% Use freqz to calculate the frequency response
[H, W] = freqz(final_array, 1, 1024, Fs);  % 1024 frequency points, half spectrum

% Bode plot: Plot magnitude and phase
figure(1);
subplot(2, 1, 1);
plot(W, abs(H));  % Magnitude in dB
% xscale("log");
ylim([0,1]);
title('Magnitude Response');
xlabel('Normalized Frequency (\times\pi rad/sample)');
ylabel('Magnitude (dB)');

%%
[audio, fs_audio]=audioread('violin.wav');
audio=resample(audio,Fs,fs_audio);
y=1:length(audio);
order = 50;
delay_line =zeros(1,order+1);

output_audio=zeros(length(audio),1);
for i=1:length(audio)
    for j=1:order
        delay_line(order+2-j) = delay_line(order+1-j);
    end
    delay_line(1)=audio(i);
    output_audio(i) = dot(delay_line,final_array);
end

%%
audio_fft = fft(audio);
audio_fft = abs(audio_fft(1:length(audio_fft)/2+1)); % Keep only the positive frequencies
out_audio = fft(output_audio); % Compute the Fast Fourier Transform
out_audio = abs(out_audio(1:length(output_audio)/2+1)); % Keep only the positive frequencies
freq = (0:length(output_audio)/2) * (Fs /length(output_audio)); % Frequency axis (Hz)
audio_dB = 20 * log10(audio_fft);
out_audio_dB = 20 * log10(out_audio);

% Plot the audio in the frequency domain
figure(2);
hold on
plot(freq, audio_dB);
plot(freq, out_audio_dB);
xscale("log");
title('Frequency Spectrum of Output and Input Audio Signal');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;
hold off;

% Plot the audio in the frequency domain
figure(3);
hold on
plot(freq, audio_fft);
plot(freq, out_audio);
xscale("log");
title('Frequency Spectrum of Output and Input Audio Signal');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
grid on;
hold off;
