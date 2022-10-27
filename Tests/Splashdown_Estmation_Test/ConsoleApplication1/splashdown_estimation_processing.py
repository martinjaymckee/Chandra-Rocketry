# -*- coding: utf-8 -*-
import os
import os.path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

def parse_value_with_units(val, func=None):
    data, _, units = val.partition(' ')
    if func == bool:
        try:
            data = int(data.strip())
        except Exception as _:
            pass
        return bool(data), None
    data = data.strip() if func is None else func(data.strip())
    return data, None if units is None else units.strip()
    
def load_splashdown_estimation_data(path):
    value_map = [
        ('t', float),
        ('flight_mode', str),
        ('flight_event', str),
        ('flight_event_latency', float),
        ('gnss_updated', bool),
        ('imu_updated', bool),
        ('prediction_updated', bool),
        ('wind_vel_x', float),
        ('wind_vel_y', float),
        ('wind_vel_z', float),        
        ('sim_pos_x', float),
        ('sim_pos_y', float),
        ('sim_pos_z', float),
        ('sim_vel_x', float),
        ('sim_vel_y', float),
        ('sim_vel_z', float),
        ('sim_accel_x', float),
        ('sim_accel_y', float),
        ('sim_accel_z', float),
        ('meas_pos_x', float),
        ('meas_pos_y', float),
        ('meas_pos_z', float),
        ('meas_pos_x_enu', float),
        ('meas_pos_y_enu', float),
        ('meas_pos_z_enu', float),        
        ('meas_accel_x', float),
        ('meas_accel_y', float),
        ('meas_accel_z', float),
        ('meas_accel_x_enu', float),
        ('meas_accel_y_enu', float),
        ('meas_accel_z_enu', float),        
        ('est_pos_x', float),
        ('est_pos_y', float),
        ('est_pos_z', float),
        ('est_pos_x_enu', float),
        ('est_pos_y_enu', float),
        ('est_pos_z_enu', float),        
        ('est_vel_x', float),
        ('est_vel_y', float),
        ('est_vel_z', float),
        ('est_vel_x_enu', float),
        ('est_vel_y_enu', float),
        ('est_vel_z_enu', float),        
        ('pred_pos_x', float),
        ('pred_pos_y', float),
        ('pred_pos_z', float),
        ('pred_pos_x_enu', float),
        ('pred_pos_y_enu', float),
        ('pred_pos_z_enu', float)        
    ]
    
    parsed_data = {}
    
    for key, _ in value_map:
        parsed_data[key] = []
        
    try:
        with open(path, 'r') as file:
            count = 0
            for line in file.readlines():
                tokens = [v.strip() for v in line.split(',')]
                parsed_data['idx'] = int(count)
                if len(tokens) == len(value_map):
                    for val, (key, func) in zip(tokens, value_map):
                        data, units = parse_value_with_units(val, func)
                        parsed_data[key].append(data)
                count = count + 1
    except Exception as e:
        print(e)
    return pd.DataFrame(parsed_data)


def splashdown_prediction_df(df, t_delay = 1.5):
    df = df[df['flight_mode'] == 'Descent'] # Only In Descent
    df = df[df['prediction_updated'] == True] # Only updated predictions
    t_apogee = df['t'].iloc[0]
    df = df[df['t'] >= (t_apogee + t_delay)] # Begining t_delay after apogee
    return df    
    

def plot_events_on_timeseries(df, axs):
    flight_event_colors = { # Colors are for the detection time, and predicted time
        'Liftoff' : ('g', 'g'),
        'Apogee' : ('c', 'c'),
        'Burnout' : ('k', 'k'),
        'Landing' : ('b', 'b')
    }
    
    for idx, (evt, latency) in enumerate(zip(df['flight_event'], df['flight_event_latency'])):
        if not evt == 'None':
            t = df['t'].at[idx]
            for ax in axs: # TODO: FIGURE OUT HOW TO LABEL THE EVENTS
                # Predicted Time
                ax.axvline(t-latency, c=flight_event_colors[evt][1], alpha=0.25)

                # Detection Time
                ax.axvline(t, c=flight_event_colors[evt][0], alpha=0.75)
                

def plot_prediction_time(df, axs):
    for t in df['t']:
        for ax in axs:
            ax.axvline(t, c='k', alpha=0.1)


def main():
    filename = 'splashdown_sim.dat'
    
    df = load_splashdown_estimation_data(filename)
    df2 = df[df['prediction_updated'] == True] # Only updated predictions
    df3 = splashdown_prediction_df(df)

    
    fig, axs = plt.subplots(3, sharex=True, constrained_layout=True)
    fig.suptitle('Velocities')    
    axs[0].set_ylabel('Vx (m/s)')
    axs[1].set_ylabel('Vy (m/s)')    
    axs[2].set_ylabel('Vz (m/s)')    
    axs[2].set_xlabel('Time (s)')        
    axs[0].plot(df['t'], df['sim_vel_x'], c='b')
    axs[0].scatter(df2['t'], df2['est_vel_x_enu'], marker='.', c='g')
    axs[1].plot(df['t'], df['sim_vel_y'], c='b')
    axs[1].scatter(df2['t'], df2['est_vel_y_enu'], marker='.', c='g')    
    axs[2].plot(df['t'], df['sim_vel_z'], c='b')
    axs[2].scatter(df2['t'], df2['est_vel_z_enu'], marker='.', c='g')    
    plot_events_on_timeseries(df, axs)
    plot_prediction_time(df2, axs)        
        
    
    fig, axs = plt.subplots(3, sharex=True, constrained_layout=True)
    fig.suptitle('Velocity Errors')
    axs[0].set_ylabel('Vx (m/s)')
    axs[1].set_ylabel('Vy (m/s)')    
    axs[2].set_ylabel('Vz (m/s)')    
    axs[2].set_xlabel('Time (s)')        
    axs[0].plot(df2['t'], df2['est_vel_x_enu'] - df2['sim_vel_x'], c='r')
    axs[1].plot(df2['t'], df2['est_vel_y_enu'] - df2['sim_vel_y'], c='r')    
    axs[2].plot(df2['t'], df2['est_vel_z_enu'] - df2['sim_vel_z'], c='r')
    plot_events_on_timeseries(df, axs)     
    plot_prediction_time(df2, axs)        
    
    fig, axs = plt.subplots(3, sharex=True, constrained_layout=True)
    fig.suptitle('Positions')    
    axs[0].set_ylabel('Px (m)')
    axs[1].set_ylabel('Py (m)')    
    axs[2].set_ylabel('Pz (m)')    
    axs[2].set_xlabel('Time (s)')    
    axs[0].plot(df['t'], df['sim_pos_x'], c='b')
    axs[0].scatter(df2['t'], df2['meas_pos_x_enu'], marker='.', c='m')
    axs[0].scatter(df2['t'], df2['est_pos_x_enu'], marker='.', c='g')
    axs[1].plot(df['t'], df['sim_pos_y'], c='b')
    axs[1].scatter(df2['t'], df2['meas_pos_y_enu'], marker='.', c='m')    
    axs[1].scatter(df2['t'], df2['est_pos_y_enu'], marker='.', c='g')    
    axs[2].plot(df['t'], df['sim_pos_z'], c='b')
    axs[2].scatter(df2['t'], df2['meas_pos_z_enu'], marker='.', c='m')    
    axs[2].scatter(df2['t'], df2['est_pos_z_enu'], marker='.', c='g')
    plot_events_on_timeseries(df, axs)     
    plot_prediction_time(df2, axs)
    N = len(df3)
    #axs[0].scatter(df3['t'], df3['pred_pos_x_enu'], marker='.', c='c')
    ax0 = axs[0].twinx()
    ax0.set_ylabel('X Error (m)')
    x_final = df3['sim_pos_x'].iloc[N-1]
    ax0.plot(df3['t'], x_final - df3['pred_pos_x_enu'], c='r')
    #axs[1].scatter(df3['t'], df3['pred_pos_y_enu'], marker='.', c='c')
    ax1 = axs[1].twinx()
    ax1.set_ylabel('Y Error (m)')
    y_final = df3['sim_pos_y'].iloc[N-1]
    ax1.plot(df3['t'], y_final - df3['pred_pos_y_enu'], c='r')
    
    fig, ax = plt.subplots(1, constrained_layout=True)
    ax.scatter([0], [0], c='k')
    ax.scatter([x_final], [y_final], c='g')
    ax.axhline(y_final, c='g', alpha=0.25)
    ax.axvline(x_final, c='g', alpha=0.25)
    ids = np.array(df3['t'])
    xs = np.array(df3['pred_pos_x_enu'])
    ys = np.array(df3['pred_pos_y_enu'])
    ax.scatter(xs, ys, c=ids, alpha=0.33, vmin=ids[0], vmax=ids[len(ids)-1], cmap='viridis', ec=None)    
    ax.set_aspect('equal')
    
#    fig, ax = plt.subplots(1, constrained_layout=True)
#    ax.plot(df['t'], df['wind_vel_x'])
#    ax.plot(df['t'], df['wind_vel_y'])

    ts = []
    ps = []
    for t, x, y, z in zip(df3['t'], df3['pred_pos_x_enu'], df3['pred_pos_y_enu'], df3['pred_pos_z_enu']):
        ts.append(t)
        ps.append( (x, y, z) )
        
    print(df['flight_event'].unique())
    plt.show()
    return


if __name__ == '__main__':
    main()