figure;
m = 0;
sig = 1;
dim = length(m);
xlimit = 5;

x = [-xlimit : 0.1 : m - 3*sig, m - 3*sig : 0.01 : m + 3*sig, m + 3*sig: 0.1 :xlimit];
sig_inv = inv(sig);

pdf = (1/(((sqrt(2*pi))^dim)*det(sig)))*exp(-.5*(x-m).*sig_inv.*(x-m));

plot(x,pdf,'linewidth',2)
xlabel('b','fontsize',14)
ylabel('pdf over belief','fontsize',14)
set(gca,'fontsize',14)

grid on