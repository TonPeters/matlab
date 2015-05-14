function out = cost_fricPC(param,tau,tau_diff)

    
    tau_est = tau*param(1)+param(2);
    out = tau_est-tau_diff;
end