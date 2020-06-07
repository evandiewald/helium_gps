import requests
import pandas as pd
import numpy as np

# list hotspots
url = "https://explorer.helium.foundation/api/hotspots"
payload = {}
headers= {}
r = requests.get(url=url, headers=headers)
data = r.json()

rows = []
for i in range(len(data['data'])):
    hotspot_data = data['data'][i]
    rows.append(hotspot_data)

df = pd.DataFrame(rows)

# list up to last 100 rewards
earnings_breakdown = np.zeros((len(df), 5))
for j in range(len(df)):
    rewards_url = 'https://explorer.helium.foundation/api/hotspots/' + df['address'][j] + '/rewards'
    try:
        r = requests.get(rewards_url, timeout=5)
    except:
        continue
    rewards_data = r.json()
    rewards_data = rewards_data['data']
    total_earnings = 0
    poc_challengers_earnings = 0
    poc_challengees_earnings = 0
    poc_witnesses_earnings = 0
    consensus_earnings = 0
    for k in range(len(rewards_data)):
        total_earnings += rewards_data[k]['amount']
        if rewards_data[k]['type'] == 'poc_challengers_reward':
            poc_challengers_earnings += rewards_data[k]['amount']
        elif rewards_data[k]['type'] == 'poc_challengees_reward':
            poc_challengees_earnings += rewards_data[k]['amount']
        elif rewards_data[k]['type'] == 'poc_witnesses_reward':
            poc_witnesses_earnings += rewards_data[k]['amount']
        elif rewards_data[k]['type'] == 'consensus_reward':
            consensus_earnings += rewards_data[k]['amount']
        else:
            print('New reward type: ', rewards_data[k]['type'])
    earnings_breakdown[j] = [total_earnings, poc_challengers_earnings, poc_challengees_earnings, poc_witnesses_earnings, consensus_earnings]
    if np.mod(j, 100) == 0:
        print(j, ' Hotspots parsed out of ', len(df))

earnings_df = pd.DataFrame(earnings_breakdown)
earnings_df.columns = ['total_earnings', 'poc_challengers_earnings', 'poc_challengees_earnings', 'poc_witnesses_earnings', 'consensus_earnings']
full_hotspot_data = pd.concat([df, earnings_df], axis=1)

# get hotspot "ages", which are the number of months the account has been active (i.e. the balance changed)
# note that this is not a perfect metric
hotspot_age = np.zeros((len(full_hotspot_data),))
for i in range(len(full_hotspot_data)):
    url = 'https://explorer.helium.foundation/api/accounts/' + full_hotspot_data['owner'].iloc[i]
    # url = 'https://explorer.helium.foundation/api/hotspots/11VKaN7fEvDm6NaGhcZtNSU1KAQQmTSwuuJsYYEqzh8mSWkoEUd/activity'
    r = requests.get(url, timeout=5)
    data = r.json()
    data = data['data']
    if len(data) == 0:
        hotspot_age[i] = np.nan
    else:
        # get number of active months (account balance changed)
        hotspot_age[i] = len(np.nonzero(np.diff(data['history']['month']))[0])
    if np.mod(i, 100) == 0:
        print(i)

ages_df = pd.DataFrame(hotspot_age)
ages_df.columns = ['hotspot_age']
full_hotspot_data_with_ages = pd.concat([full_hotspot_data, ages_df], axis=1)
full_hotspot_data_with_ages.to_csv('hotspot_data_6_7.csv')
