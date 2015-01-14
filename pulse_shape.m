pkg load signal

fs=1e6;
wpm=60;
ramp_excess=.3;

period=1200/wpm*.001*2;
ramp_time=period/2*ramp_excess;
flat_time=period/2*(1-ramp_excess);
n_samp=round(period*fs);
n_ramp=round(ramp_time*fs);
n_flat=round(flat_time*fs);
printf('on_time = %5.2f ms\n',period/2*1000);
printf('ramp    = %5.2f ms\n',ramp_time*1000);

%ramp_func=@(x)x;
%ramp=ramp_func(linspace(0,1,n_ramp));
ramp_func=@(x)(-cos(x*pi)+1)/2;
ramp=ramp_func(linspace(0,1,n_ramp));
%ramp_func=@(x)repmat(1,1,length(x));
%ramp=ramp_func(linspace(0,1,n_ramp));
%ramp_func=@(x)exp(-(x-1).^2/(2*0.3^2));
%ramp=ramp_func(linspace(0,1,n_ramp));
%ramp=transpose(hamming(2*n_ramp+1));
%ramp=ramp(1:n_ramp);
%ramp=transpose(chebwin(2*n_ramp+1,100));
%ramp=ramp(1:n_ramp);
%ramp_func=@(x)(erf((x-0.5)*3.0)+1)/2;
%ramp=ramp_func(linspace(0,1,n_ramp));
ramp=ramp-ramp(1);
ramp=ramp/max(ramp)*1;

sig=[ramp ones(1,n_flat) fliplr(ramp) zeros(1,n_samp-n_flat-2*n_ramp)];
%sig=sig-mean(sig);

sig=round(sig*8)/8;
sig=sig+randn(1,length(sig))/10000;

f=linspace(0,2*fs,n_samp+1);
f=f(1:end-1);
m=abs((fft(sig))).^2;
m=m/max(m);
plot(f,10*log10(m));
xlim([0 2000]);
ylim([-80 0]);

tot_pwr=sum(m(1:floor(length(m)/2)));
bw_percentage=0.999;
for obw=1:length(f)
  if (sum(m(1:obw))/tot_pwr>bw_percentage)
    break;
  end
end

fprintf('TX %5.2f%% cutoff frequency offset: %6.2f Hz\n',bw_percentage*100,2*fs/n_samp*obw);

