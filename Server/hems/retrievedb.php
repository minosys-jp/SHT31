<?php
header("Content-Type: text/plain");
header("Program: no-cache");
header("Cache-Control: no-cache");

# Set your DATABASE file path
$DATABASE = "<Your DATABASE path>";
$sqlite = new SQLite3($DATABASE, SQLITE3_OPEN_READONLY);
date_default_timezone_set("Asia/Tokyo");
$offset = 0;
if (isset($_GET["offset"])) {
  $offset = $_GET["offset"];
}

$year = 0;
$month = 0;
$day = 0;
$hour = 0;
$minutes = 0;
if (isset($_GET["year"])) {
  $year = $_GET["year"];
  if (!is_numeric($year)) {
    $year = 0;
  }
}
if (isset($_GET["month"])) {
  $month = $_GET["month"];
  if (!is_numeric($month)) {
    $month = 0;
  }
}
if (isset($_GET["day"])) {
  $day = $_GET["day"];
  if (!is_numeric($day)) {
    $day = 0;
  }
}
if (isset($_GET["hour"])) {
  $hour = $_GET["hour"];
  if (!is_numeric($hour)) {
    $hour = 0;
  }
}
if (isset($_GET["minutes"])) {
  $minutes = $_GET["minutes"];
  if (!is_numeric($minutes)) {
    $minutes = 0;
  }
}


switch ($_GET["type"]) {
case "ROOM":
  show_room($sqlite);
  break;
case "COUNT":
  show_count($sqlite);
  break;
case "TRACK":
  if (array_key_exists("mode", $_GET) && $_GET["mode"] != "minute") {
    show_average($sqlite, $offset, $year, $month, $day, $hour, $minutes);
  } else {
    show_track($sqlite, $offset, $year, $month, $day, $hour, $minutes);
  }
  break;
case "STATUS":
  show_status($sqlite, $offset);
  break;
case "UPTIME":
  show_uptime($sqlite);
  break;
case "THRESHOLD":
  show_threshold($sqlite);
  break;
}
$sqlite->close();

function show_room($sqlite) {
  $results = $sqlite->query("select id, name, description from ROOM");
  while ($row = $results->fetchArray()) {
    echo sprintf("%d,%s,%s\n", $row[0], $row[1], $row[2]);
  }
}

function show_track($sqlite, $offset, $year, $month, $day, $hour, $minutes) {
  $dbname = get_dbname($sqlite, $_GET["room"]);
  $limit = 60; // 60 minutes
  if ($_GET["limit"]) {
    $limit = $_GET["limit"];
  }
  if (!$dbname) return;
  $results = "";
  if ($year && $month && $day) {
    $results = $sqlite->query(sprintf("select datetime, temperature, humidity from %s where datetime <= DATETIME('%04d-%02d-%02d %02d:%02d:00') order by elapsed desc limit %d", $dbname, $year, $month, $day, $hour, $minutes, $limit));
  } else {
    $results = $sqlite->query(sprintf("select datetime, temperature, humidity from %s order by elapsed desc limit %d, %d", $dbname, $offset, $limit));
  }
  while ($row = $results->fetchArray()) {
    echo sprintf("%s,%g,%g\n", $row[0], $row[1], $row[2]);
  }
}

function show_average($sqlite, $offset, $year, $month, $day, $hour, $minutes) {
  $dbname = get_dbname($sqlite, $_GET["room"]);
  $mode = "hour";
  $limit = 48; // 2days
  $mult = 3600;
  $sql = "";
  if ($year && $month && $day) {
    $sql = sprintf("select elapsed_hour, avg(temperature), avg(humidity) from %s where datetime <= DATETIME('%04d-%02d-%02d %02d:%02d:00') group by elapsed_hour order by elapsed_hour desc limit %d", $dbname, $year, $month, $day, $hour, $minutes, $limit);
  } else {
    $sql = sprintf("select elapsed_hour, avg(temperature), avg(humidity) from %s group by elapsed_hour order by elapsed_hour desc limit %d, %d", $dbname, $offset, $limit);
  }
  if ($_GET["mode"] == "day") {
    $mode = "day";
    $limit = 60; // 2months
    $mult = 86400;
    if ($year && $month && $day) {
      $sql = sprintf("select elapsed_day, avg(temperature), avg(humidity) from %s where datetime <= DATE('%04d-%02d-%02d') group by elapsed_day order by elapsed_day desc limit %d", $dbname, $year, $month, $day, $limit);
    } else {
      $sql = sprintf("select elapsed_day, avg(temperature), avg(humidity) from %s group by elapsed_day order by elapsed_day desc limit %d, %d", $dbname, $offset, $limit);
    }
  }

  echo $sql;
  $results = $sqlite->query($sql);
  while ($row = $results->fetchArray()) {
    $dt = new DateTime();
    $dt->setTimestamp($row[0] * $mult);
    echo sprintf("%s,%g,%g\n", $dt->format('Y-m-d H:i:s'), $row[1], $row[2]);
  }
}

function show_status($sqlite, $offset) {
  $dbname = get_dbname($sqlite, $_GET["room"]);
  if (!$dbname) return;
  $results = $sqlite->query(sprintf("select datetime, status from %s order by elapsed desc limit %d, 60", $dbname, $offset));
  while ($row = $results->fetchArray()) {
    echo sprintf("%s,%d\n", $row[0], $row[1]);
  }
}

function show_uptime($sqlite) {
  $dbname = get_dbname($sqlite, $_GET["room"]);
  if (!$dbname) return;
  $results = $sqlite->query(sprintf("select MAX(elapsed) - MIN(elapsed) from %s", $dbname));
  while ($row = $results->fetchArray()) {
    $r = $row[0];
    $day = floor($r / (3600 * 24));
    $r -= $day * 3600 * 24;
    $hour = floor($r / 3600);
    $r -= $hour * 3600;
    $minute = floor($r / 60);
    $r -= $minute * 60;
    echo sprintf("%d:%02d:%02d:%02d", $day, $hour, $minute, $r);
  }
}

function show_count($sqlite) {
  $dbname = get_dbname($sqlite, $_GET["room"]);
  if (!$dbname) return;
  $results = $sqlite->query(sprintf("select count(*) from %s", $bname));
  while ($row = $results->fetchArray()) {
    $r = $row[0];
    echo printf("%d", $r);
  }
}

function show_threshold($sqlite) {
  $results = $sqlite->query(sprintf("select distinct threshold1 from THRESHOLD where room=%d order by threshold1", $_GET['room']));
  $arr = [];
  while ($results && ($row = $results->fetchArray())) {
    $arr[] = $row[0];
  }
  echo join(',', $arr);
}

function get_dbname($sqlite, $room) {
  $results = $sqlite->query(sprintf("select tablename from SUMMARY where room=%d", $room));
  while ($row = $results->fetchArray()) {
    return $row[0];
  }
  return NULL;
}

?>

