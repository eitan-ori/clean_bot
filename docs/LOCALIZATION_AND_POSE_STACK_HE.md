# איפה הרובוט חושב שהוא — מסמך מקיף (Localization / Pose / TF)

המסמך הזה מסביר **בצורה מערכתית** את כל החלקים שמשפיעים על ההערכה של מיקום הרובוט (pose): מהליידר, SLAM (Cartographer), Nav2, TF, אודומטריה, IMU, וכן **האולטרסוניק** (ומה הוא *לא* אמור להשפיע עליו).

> מטרה: כשתראה הודעות כמו `Robot is out of bounds of the costmap!` או `planner_server worldToMap failed`, תדע בדיוק **איפה לחפש** ומי הרכיב שיכול לגרום לזה.

---

## 1) מושגים בסיסיים: Pose, Map, TF

### Pose
"Pose" הוא המיקום והכיוון של הרובוט במרחב. לרוב מתייחסים אליו כ־$(x, y, \theta)$ במערכת צירים כלשהי.

### TF (Transform Tree)
ב־ROS2, TF הוא מנגנון שמחזיק **עץ של טרנספורמים** בין פריימים (frames). כל טרנספורם אומר איך לעבור בין מערכות צירים.

ברוב המערכות של Nav2 + SLAM יש את העץ הבא (הנפוץ ביותר):

- `map` → `odom` → `base_link` → `base_laser` / חיישנים

משמעות:
- `map`: מערכת צירים גלובלית "יציבה" של המפה (SLAM / Map Server)
- `odom`: מערכת צירים מקומית שמתקדמת "חלק" בזמן קצר (אודומטריה)
- `base_link`: גוף הרובוט

**מי קובע את המיקום של הרובוט על המפה?**
- זה בעצם ה־TF: `map -> base_link` (שמגיע משילוב של `map->odom` ו־`odom->base_link`).

---

## 2) מי מייצר מה? (מקורות נתונים)

### 2.1 LiDAR (`/scan`)
הליידר הוא חיישן שנותן ענן מדידות מרחק (LaserScan). הליידר משמש ל:
- SLAM (Cartographer) כדי לבנות מפה ולתקן דריפט
- Nav2 Costmaps כדי לסמן מכשולים (obstacle layer)
- לפעמים גם ללוקליזציה על מפה קיימת (אם משתמשים ב־AMCL או scan-matching)

הליידר **לא** נותן pose ישירות; הוא נותן תצפיות שמיושרות למפה ע"י אלגוריתם (SLAM / localization).

### 2.2 אודומטריה (`/odom`) — גלגלים / rf2o / fusion
אודומטריה היא הערכת תנועה יחסית:
- "נסענו 0.2 מטר קדימה"
- "סובבנו 10 מעלות"

אודומטריה יכולה להגיע מ:
- אנקודרים בגלגלים (wheel odom)
- odometry מליידר (למשל rf2o_laser_odometry)
- איחוד IMU + wheel (EKF)

היא טובה לטווח קצר, אבל בדרך כלל **נודדת (drift)** בטווח ארוך.

### 2.3 IMU
IMU נותן תאוצות/מהירויות זוויתיות (Yaw rate) וכו'.
- משפר יציבות כיוון, בעיקר בפניות/שינויים מהירים
- לא בונה מפה
- לבדו לא מספיק ל־global localization

### 2.4 Ultrasonic (האולטרסוניק)
האולטרסוניק נותן "מרחק לקיר" בקו צר. במערכת שלנו הוא מומר בדרך כלל ל־PointCloud2 (למשל `/low_obstacles`) ומשמש ל:
- **הימנעות ממכשולים (avoidance)** דרך ה־**local_costmap**

חשוב:
- הוא **לא אמור להזיז את pose**
- הוא **לא אמור להשפיע על `/map`**
- הוא **לא אמור להזין Cartographer**

אם האולטרסוניק היה משפיע על pose בפועל, זה היה אומר שאתה עושה sensor fusion ל־localization (למשל EKF עם מדידה מוחלטת), וזה לא מה שקורה כאן.

---

## 3) Cartographer: איך הוא קובע איפה הרובוט על המפה

Cartographer (במצב 2D) עושה בערך את זה:
1. מקבל `/scan` + TF של הליידר ביחס ל־`base_link` (למשל `base_link -> laser`)
2. משתמש באודומטריה/IMU (אם מוגדר) כ־prior לתנועה
3. עושה **scan matching**: מתאים את הסריקה למפה/תת־מפה (submap)
4. משפר את ההערכה של pose ומעדכן את ה־TF

ברוב ההתקנות:
- `cartographer_node` מפרסם **TF `map -> odom`**
- האודומטריה מפרסמת **TF `odom -> base_link`**
- יחד זה נותן `map -> base_link`

בנוסף:
- `cartographer_occupancy_grid_node` מפרסם את `/map` (OccupancyGrid)

### 3.1 למה "מחוץ למפה" קורה עם Cartographer?
זה קורה כשה־pose (`map->base_link`) או היעד (goal) יוצאים מחוץ לגבולות של:
- ה־OccupancyGrid (`/map.info.origin + width/height/resolution`)
- או ה־costmap window (global/local)

דוגמא קלאסית:
- המפה קטנה/חתוכה (למשל רק האזור שנסרק עד עכשיו)
- הרובוט/goal ליד הקצה
- ה־planner בודק נקודות במסלול שיוצאות תא/שניים מעבר -> `worldToMap failed`

זה **לא** קשור לזה ש"Nav2 לא עומד בעדכוני מפה". Nav2 כן עובד עם מפה שמתעדכנת; הבעיה היא לרוב goals/pose בקצה או TF לא עקבי.

---

## 4) Nav2: איך הוא משתמש במיקום ובמפה

Nav2 לא עושה SLAM (ברירת מחדל). Nav2 מניח שיש לו:
- `/map` (OccupancyGrid)
- TF תקין `map -> odom -> base_link`
- חיישנים לקוסטמפ (למשל `/scan`)

### 4.1 Global costmap
ה־global costmap בד"כ עובד ב־`global_frame: map`.
הוא משתמש ב־`/map` כבסיס סטטי + שכבות מכשולים.

פרמטרים קריטיים:
- `rolling_window`: אם True, החלון זז סביב הרובוט (יותר קל לרובוט קטן, אבל מסוכן אם goals רחוקים)
- `width/height`: גודל החלון במטרים
- `resolution`: גודל תא (במטרים)
- `origin_x/origin_y`: איפה החלון מתחיל

אם אתה מקבל `worldToMap failed` עם `mx,my` גדולים מ־`size_x,size_y`, זה אומר ש־Nav2 מנסה לשאול תא שלא קיים בחלון.

### 4.2 Local costmap
ה־local costmap עובד ב־`global_frame: odom` (או base) והוא לחלון קטן סביב הרובוט.
הוא אמור לכלול חיישנים כמו:
- `/scan`
- `/low_obstacles` (אולטרסוניק כ־PointCloud2)

כאן בדיוק המקום הנכון לאולטרסוניק: הימנעות ממכשולים "נמוכים".

### 4.3 Planner / Controller
- Planner (למשל NavFn / Smac) מתכנן מסלול על global costmap
- Controller (DWB / RPP) עוקב אחר המסלול ומפיק `/cmd_vel`

שגיאות מהסוג `planner_server worldToMap failed` הן *כמעט תמיד*:
- יעד מחוץ למפה/חלון costmap
- TF `map->base_link` לא הגיוני
- או חוסר תאימות/קפיצה של מקור `/map`

---

## 5) נקודה קריטית: אסור שיהיו שני Publishers על `/map`

אם יש יותר מ־publisher אחד ל־`/map`, Nav2 יכול "להחליף" בין מפות:
- פעם אחת origin/size א'
- פעם אחת origin/size ב'

זה יוצר בדיוק:
- `worldToMap failed`
- `Robot is out of bounds of the costmap`

במערכת שלנו הייתה נקודה כזאת בעבר: `web_control_panel` היה יכול לפרסם `/map` בנוסף לקרטוגרפר.

**המצב התקין:**
- `Publisher count: 1`
- המפרסם הוא `cartographer_occupancy_grid_node`

פקודה:
```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop; ros2 daemon start
ros2 topic info --verbose /map | sed -n '1,80p'
```

---

## 6) ROS_DOMAIN_ID ו־"למה אני לא רואה נודים"

אם אתה מריץ `ros2 node list` ורואה רק חלק מהנודים, יש 2 סיבות נפוצות:
1. אתה בדומיין הלא נכון (`ROS_DOMAIN_ID` לא תואם)
2. `ros2 daemon` תקוע/מחזיק cache לא מעודכן

תיקון:
```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop
ros2 daemon start
```

ואז שוב:
```bash
ros2 node list | head
ros2 topic list | egrep 'map|tf'
```

---

## 7) איך לאבחן בפועל "איפה הרובוט חושב שהוא"

### 7.1 בדיקת גבולות המפה
צריך להשוות את pose לגבולות OccupancyGrid:
- גבולות X:
  - $x \in [origin_x, origin_x + width \cdot resolution]$
- גבולות Y:
  - $y \in [origin_y, origin_y + height \cdot resolution]$

כדי לראות metadata בלי להדפיס את כל `data`:
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo -n 1 /map --no-arr | sed -n '1,80p'
```

### 7.2 בדיקת TF של הרובוט על המפה
בדוק אם קיים טרנספורם:
```bash
export ROS_DOMAIN_ID=42
ros2 run tf2_ros tf2_echo map base_link
```

אם אין TF, Nav2 לא יכול לתכנן במפה.

### 7.3 בדיקת פרמטרים של global_costmap בזמן ריצה
```bash
export ROS_DOMAIN_ID=42
ros2 param get /global_costmap/global_costmap rolling_window
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap resolution
ros2 param get /global_costmap/global_costmap origin_x
ros2 param get /global_costmap/global_costmap origin_y
```

### 7.4 להבין את הודעת השגיאה `worldToMap failed`
דוגמא:
```
worldToMap failed: mx,my: 175,60, size_x,size_y: 160,61
```
פירוש:
- המרה מעולם לתאים נתנה col=175 אבל למפה יש רק 0..159
- כלומר הנקודה (או דגימה סביב goal) נמצאת *מחוץ לחלון*.

זה קורה לרוב כאשר:
- היעד נבחר על שפת המפה
- או שהמפה עדיין לא "גדלה" לאזור הזה
- או שהחלון של costmap קטן מדי / origin לא ממוקם טוב

---

## 8) איך האולטרסוניק כן משפיע — ואיפה זה תקין

האולטרסוניק צריך להיכנס ל־**local_costmap** בלבד:
- marking (או marking+clearing בהתאם לרעש)
- כדי למנוע פגיעה במכשולים נמוכים

הוא לא צריך:
- לפרסם TF
- לפרסם `/map`
- להזין Cartographer
- להשפיע על pose (`map->base_link`)

אם אתה רואה שהרובוט "זז" על המפה בגלל אולטרסוניק — זה כמעט תמיד:
- בעיית TF / ליידר / אודום
- או מפה שמתחלפת (multi publisher)
- או שליחת goals לקצה

---

## 9) תרחישים נפוצים וסימפטומים

### תרחיש A: TF לא יציב / חסר
סימנים:
- Nav2 לא זז
- `tf2_echo map base_link` נכשל
- לוגים על missing transform

פתרון:
- לוודא `robot_state_publisher` רץ
- לוודא `odom->base_link` מתפרסם
- לוודא Cartographer מפרסם `map->odom`

### תרחיש B: שני publishers על `/map`
סימנים:
- `Publisher count: 2`
- `worldToMap failed`/out-of-bounds מופיע "ללא היגיון" ומתחיל פתאום

פתרון:
- להשאיר publisher יחיד (Cartographer) על `/map`

### תרחיש C: goals על שפת המפה בזמן exploration
סימנים:
- ספאם `worldToMap failed` בזמן סריקה/פרונטירים
- במיוחד כשהרובוט ליד גבולות האזור הסרוק

פתרון:
- לא לשלוח goals קרוב לשפה
- להוסיף margin goals
- להבטיח שהיעד נמצא בתוך free space

---

## 10) פקודות שימושיות (Cheat Sheet)

```bash
# לעבוד בדומיין הנכון
export ROS_DOMAIN_ID=42

# לרענן discovery
ros2 daemon stop; ros2 daemon start

# לראות מי מפרסם מפה
ros2 topic info --verbose /map | sed -n '1,80p'

# לראות metadata של המפה
ros2 topic echo -n 1 /map --no-arr | sed -n '1,80p'

# לראות TF של הרובוט על המפה
ros2 run tf2_ros tf2_echo map base_link

# פרמטרי global costmap
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap rolling_window
```

---

## 11) הערות ספציפיות לפרויקט הזה

- מקור `/map` התקין הוא `cartographer_occupancy_grid_node`.
- Nav2 משתמש ב־`/map` + TF כדי לתכנן.
- האולטרסוניק (`/low_obstacles`) אמור להשפיע על **local_costmap בלבד**.
- אם חוזרת בעיית `worldToMap failed` בזמן exploration, לעיתים זה לא "מפה משתנה" אלא פשוט goals קרובים מדי לשפה. במקרה כזה הפתרון הוא להקשיח את בחירת היעד (margin ובדיקה של free space).

---

אם תרצה, אני יכול גם להוסיף למסמך הזה סעיף "מי מפרסם איזה TF" לפי ה־launch שלך (שמות נודים/טופיקים אמיתיים אצלך) — פשוט שלח:
- `ros2 topic info --verbose /tf | sed -n '1,80p'`
- `ros2 topic info --verbose /tf_static | sed -n '1,80p'`
- `ros2 run tf2_tools view_frames` (אם יש) או צילום מסך מה־TF tree ב־RViz.
