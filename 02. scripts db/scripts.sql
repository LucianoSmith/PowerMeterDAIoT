CREATE TABLE `daiot-pm.monitor_data_db.powermeter`
(
  timestamp TIMESTAMP,
  gcp_device_id STRING,
  dev_id STRING,
  volt FLOAT64,
  curr FLOAT64,
  rssi INT64,
  wake_up_count INT64,
  last_error_count INT64,
  last_error_code INT64,
  last_on_time INT64,
  sntp_response_time INT64,
  led1 STRING OPTIONS(description="Estado Led 1 (rojo)"),
  led2 STRING OPTIONS(description="Estado Led 2 (amarillo)"),
  led3 STRING OPTIONS(description="Estado Led 3 (verde)"),
  rele STRING OPTIONS(description="Estado el rele")
);

CREATE VIEW `daiot-pm.monitor_data_db.last_reg` AS
SELECT * FROM `daiot-pm.monitor_data_db.powermeter` pm
WHERE pm.timestamp = (select max(pm2.timestamp) from `daiot-pm.monitor_data_db.powermeter` pm2);

CREATE TABLE `daiot-pm.monitor_data_db.devices`
(
dev_id STRING,
ubicacion STRING,
propietario STRING,
descripcion STRING,
curr_max FLOAT64,
volt_max FLOAT64,
volt_min FLOAT64
);

