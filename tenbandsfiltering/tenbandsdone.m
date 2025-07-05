gain=zeros(10,1);%[1 2 3 4 5 6 7 8 9 10];
load('filter1.mat');
load('filter2.mat');
load('filter3.mat');
load('filter4.mat');
load('filter5.mat');
load('filter6.mat');
load('filter7.mat');
load('filter8.mat');
load('filter9.mat');
load('filter10.mat');
filter_coeff=[filter1 ; filter2 ; filter3 ; filter4 ; filter5 ; filter6 ; filter7 ; filter8 ; filter9 ; filter10];
final_array = zeros(51,1);
gain(2)=10;
gain(10)=10;
for i=1:10
    final_array=final_array+gain(i)*filter_coeff(i,:)
end
Fs = 44100;  % Sampling frequency 44.1 kHz
final_array=final_array(1,:)
% Use freqz to calculate the frequency response
[H, W] = freqz(final_array, 1, 1024, Fs);  % 1024 frequency points, half spectrum

% Bode plot: Plot magnitude and phase
figure(1);
subplot(2, 1, 1);
plot(W, abs(H));  % Magnitude in dB
% xscale("log");
% ylim([0,1]);
title('Magnitude Response');
xlabel('Normalized Frequency (\times\pi rad/sample)');
ylabel('Magnitude (dB)');

%%
[audio, fs_audio]=audioread('trialmusic.wav');
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
