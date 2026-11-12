"""Rebuild corrected group-count map and energy time history from the uploaded CSV."""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "openfast_72"
FIG = ROOT / "figures"

def main() -> None:
    FIG.mkdir(exist_ok=True,parents=True)
    df = pd.read_csv(DATA / "search_results.csv")
    grouped = df.groupby(["yaw_deg", "on_s"]).agg(best=("delta_gen_kwh", "max"), num=("delta_gen_kwh", "size"))
    yaw_values, on_values = [-3.0,0.0,3.0], [15.0,30.0,45.0]
    energy = np.array([[grouped.loc[(y,t),"best"] for t in on_values] for y in yaw_values])
    counts = np.array([[grouped.loc[(y,t),"num"] for t in on_values] for y in yaw_values])
    assert counts.sum()==72
    fig, axs = plt.subplots(1,2,figsize=(13.6,4.5),layout="constrained")
    vmin,vmax=0,float(np.ceil(energy.max()*2)/2)
    p0=axs[0].imshow(energy,origin="lower",aspect="auto",cmap="YlOrRd",vmin=vmin,vmax=vmax)
    fig.colorbar(p0,ax=axs[0],fraction=.05,pad=.025).set_label("Max ΔEgen [kWh]")
    p1=axs[1].imshow(counts,origin="lower",aspect="auto",cmap="Greys",vmin=0,vmax=12)
    fig.colorbar(p1,ax=axs[1],fraction=.05,pad=.025).set_label("Evaluated paths [count]")
    for ax,title in zip(axs,["Maximum eligible energy difference per cell","Evaluated paths per cell (corrected)"]):
        ax.set_title(title,fontsize=12,pad=12)
        ax.set_xlabel("Control start [s]");ax.set_ylabel("Yaw offset target [deg]")
        ax.set_xticks(range(3),[str(int(x)) for x in on_values]);ax.set_yticks(range(3),[str(int(x)) for x in yaw_values])
    for i in range(3):
        for j in range(3):
            axs[0].text(j,i,f"{energy[i,j]:+.3f}",ha="center",va="center",fontsize=12,color="white" if energy[i,j]>=2.5 else "black")
            axs[1].text(j,i,str(counts[i,j]),ha="center",va="center",fontsize=12,color="white" if counts[i,j]>=8 else "black")
    fig.savefig(FIG/"openfast_72_advantage_map_corrected.png",dpi=168,bbox_inches="tight")
    plt.close(fig)
    best=pd.read_csv(DATA/"best_timeseries.csv")
    worst=pd.read_csv(DATA/"worst_timeseries.csv")
    fig,axs=plt.subplots(2,1,figsize=(10.8,5.8),sharex=True,layout="constrained")
    for ax in axs:
        ax.axvspan(40,80,color="0.86",alpha=.6,label="Wind transition")
        ax.axhline(0,color="0.55",lw=.8)
        ax.grid(alpha=.18)
    axs[0].plot(best.time_s,best.delta_cumulative_kwh,label="Best path 0050",lw=2)
    axs[0].plot(worst.time_s,worst.delta_cumulative_kwh,label="Worst path 0028",lw=2)
    axs[0].set_ylabel("Cumulative ΔEgen [kWh]");axs[0].legend(loc="center right")
    axs[0].set_title("Generator energy difference relative to the same ROSCO baseline")
    axs[1].plot(best.time_s,best.candidate_power_kw,label="Best path 0050",lw=1.2)
    axs[1].plot(worst.time_s,worst.candidate_power_kw,label="Worst path 0028",lw=1.2)
    axs[1].plot(best.time_s,best.baseline_power_kw,label="ROSCO baseline",lw=1.2,ls="--",alpha=.75)
    axs[1].set_ylabel("Generator power [kW]");axs[1].set_xlabel("Time [s]")
    axs[1].legend(loc="lower right",ncol=3)
    fig.savefig(FIG/"openfast_72_energy_time_history.png",dpi=162,bbox_inches="tight")
    plt.close(fig)
    print("Generated corrected map and best/worst energy history")

if __name__ == "__main__":
    main()
