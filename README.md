# SMC - SimBa Mission Control

## Description
Mission control software for SimBa rockets
that runs on dockerized environment which consists of:
- InfluxDB (time-series database) 
- Worker (python script receving and writing messages into db)
- Grafana (GUI app)

## Setup
- Clone the repo: https://gitlab.com/SimLE/simba/gse/smc 
- Change IP in **.env** file 
- run `sudo docker compose up -d`
- localhost:8086 : **InfluxDB** *(simba, simba123)*
- localhost:3001 : **Grafana**

## IP adresses
- Moto G5 S: 192.168.43.153
- Home router and static ip: 192.168.0.18 

## Technologies

### InfluxDB
When you login to Influx UI (locahost:8086) in Data Explorer 
you can see all telemetry that is being send from Pixhawk.


### Grafana


## Dialect
To generate custom dialect using **.xml** file you need to:
- clone the **pymavlink** repo: https://github.com/ArduPilot/pymavlink
(cause using **pip install** doesn't install all nessessary files)
- Then from the **tools** directory run command:
<pre><code>sudo python mavgen.py -o output_file --lang=Python path_to_xml_file</code></pre>
- It will generate python file that together with **.xml** file needs to be copied to pymavlink package directory (**/usr/local/lib/python3.8/dist-packages/pymavlink/dialects**)
- Lastly copy **.xml** file to message definitions folder (**/usr/local/lib/python3.8/dist-packages/pymavlink/message_definitions/v1.0**)

## Contributing
1. Stash your changes `git stash save`
2. Get latest version `git pull`
3. Create new branch `git checkout -b <name_of_branch>`
4. Add your changes to that branch `git stash pop`
5. Stage desired changes `git add .`
6. Describe what changes you've done `git commit -s`
7. Create merge request `git push --set-upstream origin <name_of_branch>`

