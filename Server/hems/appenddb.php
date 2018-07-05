<?php
  // TODO: authentication

  $msg = "";
  $result = "NG";
  $room_hash = NULL;
  $dbtable_hash = NULL;
  $configs = NULL;

  $DATABASE = "/home/minosys/tmp/Temperatures.db";
  $sqlite = new SQLite3($DATABASE);
  create_configs($sqlite);

  $json = file_get_contents("php://input");
  //$json = '[{"current": 100, "datetime": "2014-11-10 19:00:01", "from": 60, "presence": 128, "status": 0, "temperature": 18.5, "humidity": 69.33}]';
  $jsons = json_decode($json);
  if (is_array($jsons)) {
    $count = count($jsons);
    create_dbtable_hash($sqlite);
    for ($i = 0; $i < $count; ++$i) {
      $json = $jsons[$i];
      $dbtable = $dbtable_hash[$json->{'from'}];
      $result = insert_data($sqlite, $json, $dbtable);      
    }
    if ($msg) {
      $headers  = "From:" . $configs["from"] . "\r\n";
      $headers .= "To:" . $configs["mail-to"];
      mail($configs["to"], $configs["subject"], $configs["mail-body"] . "\n" . $msg, $headers);
    }
    $sqlite->close();
  }

  function insert_data($sqlite, $json, $dbtable) {
    $c = $json->{'current'};
    $sql = sprintf("insert into %s (elapsed, elapsed_hour, elapsed_day, datetime, status, temperature, humidity) values (%d, %d, %d, '%s', %d, %g, %g)", $dbtable, $c, $c / 3600, $c / 86400, $json->{'datetime'}, $json->{'status'}, $json->{'temperature'}, $json->{'humidity'});
    if (!$sqlite->exec($sql)) {
      return "NG";
    }
    add_mail_msg($sqlite, $json->{'from'}, $json->{'temperature'});
    return "OK";
  }

  function add_mail_msg($sqlite, $from, $temperature) {
    global $msg, $room_hash;
// beep on 'down' threshold
    $sql = sprintf("select id, threshold1 from THRESHOLD where room=%d and mode='down' and current=0 and %g <= threshold1", $from, $temperature);
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $sql = sprintf("update THRESHOLD set current=1 where id=%d", $row[0]);
      $sqlite->exec($sql);
      if (!$room_hash) {
        create_room_hash($sqlite);
      }
      $msg .= sprintf("Temperature down to %g degree (threshold is %g) for %s\n", $temperature, $row[1], $room_hash[$from]);
    }

// rewind beeper
    $sql = sprintf("select id from THRESHOLD where room=%d and mode='down' and current=1 and %g > threshold2", $from, $temperature);
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $sql = sprintf("update THRESHOLD set current=0 where id=%d", $row[0]);
      $sqlite->exec($sql);
    }

// beep on 'up' threshold
    $sql = sprintf("select id, threshold1 from THRESHOLD where room=%d and mode='up' and current=0 and %g >= threshold1", $from, $temperature);
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $sql = sprintf("update THRESHOLD set current=1 where id=%d", $row[0]);
      $sqlite->exec($sql);
      if (!$room_hash) {
        create_room_hash($sqlite);
      }
      $msg .= sprintf("Temperature up to %g degree (threshold is %g) for %s\n", $temperature, $row[1], $room_hash[$from]);
    }

// rewind beeper
    $sql = sprintf("select id from THRESHOLD where room=%d and mode='up' and current=1 and %g < threshold2", $from, $temperature);
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $sql = sprintf("update THRESHOLD set current=0 where id=%d", $row[0]);
      $sqlite->exec($sql);
    }
  }

  function create_room_hash($sqlite) {
    global $room_hash;
    $sql = "select id,name from ROOM";
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $room_hash[$row[0]] = $row[1];
    }
  }

  function create_dbtable_hash($sqlite) {
    global $dbtable_hash;
    $sql = "select room, tablename from SUMMARY";
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $dbtable_hash[$row[0]] = $row[1];
    }
  }

  function create_configs($sqlite) {
    global $configs;
    $sql = "select name, value from CONFIGURATION";
    $results = $sqlite->query($sql);
    while ($results && ($row = $results->fetchArray())) {
      $configs[$row[0]] = $row[1];
    }
  }
?>
<html>
<head>
<title>result</title>
</head>
<body>
<result><?php echo $msg; ?></result>
</body>
</html>
