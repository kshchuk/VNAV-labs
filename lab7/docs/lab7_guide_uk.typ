// Лабораторна 7 (MIT 16.485 VNAV): покроковий гайд українською.
// Компіляція (з каталогу docs): typst compile lab7_guide_uk.typ

#import "@preview/cetz:0.4.2": canvas, draw

#set document(title: "VNAV Lab 7 — GTSAM і факторні графи: покроковий гайд", author: "Yarik Kishchuk")
#set page(paper: "a4", margin: (x: 2.2cm, y: 2.3cm), numbering: "1")
#set text(lang: "uk", font: "New Computer Modern", size: 10.5pt)
#set par(justify: true, leading: 0.62em)
#set heading(numbering: "1.1")
#set math.equation(numbering: "(1)", supplement: none)
#show heading.where(level: 1): it => { pagebreak(weak: true); v(0.4em); it; v(0.3em) }
#show raw: set text(font: "DejaVu Sans Mono", size: 8.3pt)
#show raw.where(block: true): block.with(fill: luma(246), inset: 8pt, radius: 3pt, width: 100%, stroke: 0.4pt + luma(210))
#show link: set text(fill: rgb("#1d4ed8"))

#let accent = rgb("#2563eb")
#let callout(title, body, color: accent) = block(
  width: 100%, inset: 9pt, radius: 3pt,
  fill: color.lighten(90%), stroke: (left: 3pt + color),
)[#text(weight: "bold", fill: color.darken(20%))[#title] \ #body]
#let note(body) = callout("Примітка", body)
#let warn(body) = callout("Пастка", body, color: rgb("#dc2626"))
#let tip(body) = callout("Порада", body, color: rgb("#059669"))
// Блок "де:" з поясненням кожної змінної формули: #vars([$x$], [опис], ...)
#let vars(..items) = block(
  width: 100%, inset: (left: 10pt, y: 5pt), stroke: (left: 1.5pt + luma(190)), above: 0.6em, below: 1em,
)[
  #set text(size: 9.5pt)
  #set par(justify: false)
  #text(fill: luma(90), style: "italic")[де:]
  #grid(columns: (auto, 1fr), column-gutter: 0.9em, row-gutter: 0.55em, ..items.pos())
]
// Таблиця змінних коду: #codevars(([`name`], [тип], [опис]), ...)
#let codevars(..rows) = { set par(justify: false); set text(size: 9.5pt); table(
  columns: (27%, 27%, 46%), inset: 5pt, stroke: 0.4pt + luma(200),
  fill: (_, y) => if y == 0 { luma(235) },
  [*Змінна*], [*Тип*], [*Що означає*],
  ..rows.pos().flatten(),
) }
#let deliv(n, pts, body) = callout([📨 #n (#pts)], body, color: rgb("#7c3aed"))

// ───────────────────────── Титульна сторінка ─────────────────────────
#align(center)[
  #v(3cm)
  #text(size: 12pt, fill: luma(90))[MIT 16.485 · Visual Navigation for Autonomous Vehicles]
  #v(0.6cm)
  #text(size: 26pt, weight: "bold")[Лабораторна 7]
  #v(0.2cm)
  #text(size: 17pt)[Нелінійні найменші квадрати, групи Лі та GTSAM]
  #v(0.8cm)
  #text(size: 12pt)[Покроковий гайд з виконання: теорія, математика, код, результати]
  #v(1.5cm)
  #block(width: 80%)[
    #set align(left)
    #set par(justify: false)
    *Що всередині:*
    - MAP-оцінка → нелінійні найменші квадрати; Гаусс–Ньютон і Левенберг–Марквардт;
    - оптимізація на многовидах $"SO"(3)$ / $"SE"(3)$: експонента, ретракція, якобіани в GTSAM;
    - індивідуальні Deliverables 1–2: зважені НМК, хордальне усереднення поз, доведення ML для Langevin + Gauss, кроки GN/LM з явними якобіанами;
    - командні Deliverables 1–5: перевірений код для GTSAM 4.2 (Docker-образ курсу) і реальні результати прогонів;
    - пастки: порядок координат у `Pose3`, калібрувальна свобода (gauge), RViz2 на macOS (Foxglove замість нього).
  ]
  #v(1fr)
  #text(size: 9pt, fill: luma(110))[Оригінальне завдання: #link("https://vnav.mit.edu/labs/lab7/exercises.html")[vnav.mit.edu/labs/lab7/exercises.html] · GTSAM primer: #link("https://gtsam.org/tutorials/intro.html")[gtsam.org/tutorials/intro.html]]
]
#pagebreak()

#outline(indent: auto, depth: 2)

// ═════════════════════════════════════════════════════════════════════
= Огляд лабораторної

Lab 7 — про те, як *формулювати* задачі оцінювання стану у вигляді нелінійних найменших квадратів (НМК) і розв'язувати їх бібліотекою GTSAM. Усі наступні лаби (VIO, SLAM) будуються саме на цьому.

#table(
  columns: (auto, 1fr, auto, auto),
  inset: 6pt, stroke: 0.4pt + luma(200),
  fill: (_, y) => if y == 0 { luma(235) },
  [*№*], [*Завдання*], [*Тип*], [*Бали*],
  [I-1], [Коваріації вимірювань у НМК: зважені НМК → стандартні, нормальні рівняння], [індивідуальне], [10],
  [I-2], [Групи Лі: хордальне усереднення $"SE"(3)$, зважена версія, доведення ML, кроки GN/LM (+5 за якобіани)], [індивідуальне], [35],
  [T-1], [Вступ до GTSAM: `Pose2`, odometry + "GPS"], [командне], [10],
  [T-2], [3D pose graph: `BetweenFactor<Pose3>` + prior, 1/3/6 ітерацій GN], [командне], [10],
  [T-3], [Власний фактор MoCap (позиція $in RR^3$) з якобіаном], [командне], [15],
  [T-4], [Bundle adjustment: `GenericProjectionFactor`], [командне], [20],
  [T-5], [(опц.) Усереднення $"SO"(3)$: власний Frobenius-фактор], [командне], [+15],
)

*Файли* (`VNAV-labs/lab7/`):
#table(
  columns: (auto, 1fr),
  inset: 5pt, stroke: 0.4pt + luma(200),
  [`src/deliverable_1.cpp`], [T-1: `UnaryFactor` (вже написаний) + TODO 1a/1b/1c],
  [`include/deliverable_2_3.h`], [T-3: `MoCapPosition3Factor` (TODO 3a)],
  [`src/deliverable_2_3.cpp`], [T-2/T-3: генерація даних, TODO 2a/2b/3b, візуалізація],
  [`src/deliverable_4.cpp`, `include/example_data.h`], [T-4: камери навколо куба, TODO — фактори й priors],
  [`include/deliverable_5.h`, `src/deliverable_5.cpp`], [T-5: Frobenius-фактор і оптимізація],
  [`launch/*.launch.yaml`, `rviz/*.rviz`], [запуск вузлів + RViz2],
)

== Позначення

- $x in RR^n$ — вектор невідомих (для евклідових задач); $X$ — набір усіх змінних факторного графа (пози, точки).
- $r(x) in RR^m$ — вектор залишків (residuals); $J = partial r \/ partial x in RR^(m times n)$ — якобіан.
- $norm(bold(e))^2_Sigma = bold(e)^top Sigma^(-1) bold(e)$ — квадрат *махаланобісової* норми з коваріацією $Sigma$.
- $R in "SO"(3)$ — обертання, $T = mat(R, bold(t); bold(0)^top, 1) in "SE"(3)$ — поза. $[bold(omega)]_times$ — кососиметрична матриця векторного добутку; $bold(e)_k$ — $k$-й базисний вектор $RR^3$.
- $"Exp": RR^3 arrow.r "SO"(3)$, $"Log"$ — експонента й логарифм групи ("великими літерами" — з урахуванням $(dot)^and$ / $(dot)^or$).
- $op("vec")(M)$ — матриця, записана в стовпець (по стовпцях), $norm(M)_F$ — норма Фробеніуса, $op("tr")$ — слід.
- Після ключових формул — блок *"де:"* з поясненням змінних; у кінці — словник позначень (додаток A).

// ═════════════════════════════════════════════════════════════════════
= Математичне підґрунтя

== Від MAP до нелінійних найменших квадратів

Маємо невідомі $X$ і вимірювання $z_1, dots, z_K$. MAP-оцінка:
$ X_"MAP" = op("arg max", limits: #true)_X p(X | Z) = op("arg max", limits: #true)_X p(Z | X) p(X) = op("arg max", limits: #true)_X product_i p(z_i | X_i) dot p(X). $
Якщо кожне вимірювання — нелінійна функція частини змінних плюс гаусів шум, $z_i = h_i(X_i) + bold(epsilon)_i$, $bold(epsilon)_i tilde cal(N)(0, Sigma_i)$, то
$ p(z_i | X_i) = 1 / sqrt((2 pi)^(d_i) det Sigma_i) exp(-1/2 norm(h_i (X_i) - z_i)^2_(Sigma_i)). $
Логарифм — монотонна функція, тож максимізація добутку = мінімізація суми від'ємних логарифмів; нормувальні множники від $X$ не залежать:
$ X_"MAP" = op("arg min", limits: #true)_X sum_i norm(h_i (X_i) - z_i)^2_(Sigma_i). $ <eq-map>
#vars(
  [$X$, $X_i$], [усі невідомі / підмножина змінних, від якої залежить $i$-те вимірювання],
  [$z_i$], [$i$-те вимірювання (відносна поза, GPS-позиція, піксель тощо)],
  [$h_i$], [модель вимірювання: яке $z_i$ ми *очікуємо* побачити при значенні $X_i$],
  [$bold(epsilon)_i$, $Sigma_i$], [шум вимірювання та його коваріація ($d_i times d_i$)],
  [$d_i$], [розмірність $i$-го вимірювання],
  [$p(X)$], [апріорний розподіл (у GTSAM — prior-фактори, тобто теж доданки в сумі)],
)
Кожен доданок @eq-map — *фактор*. Сукупність змінних і факторів — *факторний граф*: вершини-змінні, вершини-фактори, ребра "фактор залежить від змінної". GTSAM зберігає саме таку структуру (`NonlinearFactorGraph`) і використовує її розрідженість.

== Гаусс–Ньютон і Левенберг–Марквардт

Розглянемо $min_x f(x) = norm(r(x))^2$. Лінеаризуємо залишок у поточній точці: $r(x + bold(delta)) approx r(x) + J bold(delta)$. Тоді
$ f(x + bold(delta)) approx norm(r + J bold(delta))^2 = r^top r + 2 bold(delta)^top J^top r + bold(delta)^top J^top J bold(delta). $
Похідна за $bold(delta)$ дорівнює нулю, звідки *нормальні рівняння* Гаусса–Ньютона:
$ J^top J bold(delta) = -J^top r, quad x arrow.l x + bold(delta). $ <eq-gn>
*Левенберг–Марквардт* додає демпфування:
$ (J^top J + lambda D) bold(delta) = -J^top r, $ <eq-lm>
де $D = I$ або $D = op("diag")(J^top J)$. При $lambda arrow.r 0$ — крок GN (швидка квадратична збіжність біля мінімуму); при великому $lambda$ — короткий крок уздовж антиградієнта $-J^top r \/ lambda$ (надійно далеко від мінімуму). Якщо крок зменшив $f$ — приймаємо і зменшуємо $lambda$ (наприклад, у 10 разів), інакше відкидаємо і збільшуємо $lambda$.
#vars(
  [$f(x)$], [цільова функція — сума квадратів залишків],
  [$bold(delta)$], [крок (поправка) до поточного значення $x$],
  [$J^top J$], [наближення Гаусса–Ньютона до гессіана $f$ (без других похідних $r$)],
  [$-J^top r$], [антиградієнт $f$ (з точністю до множника 2)],
  [$lambda$], [параметр демпфування LM (у GTSAM — `lambdaInitial`, `lambdaFactor`)],
  [$D$], [матриця масштабування демпфування],
)

== Оптимізація на многовидах: $"SO"(3)$ і $"SE"(3)$

Пози й обертання не є векторами: $R + bold(delta)$ — не обертання. Тому крок робимо в *дотичному просторі* і "накручуємо" його на групу *ретракцією*. GTSAM використовує праве збурення:
$ R ⊕ bold(omega) = R "Exp"(bold(omega)), quad T ⊕ bold(xi) = T "Exp"(bold(xi)), quad bold(xi) = mat(bold(omega); bold(v)) in RR^6. $ <eq-retract>
Для малих приростів $"Exp"(bold(omega)) approx I + [bold(omega)]_times$, а для пози
$ T "Exp"(bold(xi)) approx (R (I + [bold(omega)]_times), med bold(t) + R bold(v)). $ <eq-pose-first-order>
*Якобіан на многовиді* — похідна за локальними координатами в нулі:
$ H = lr((partial bold(e)(X ⊕ bold(delta))) / (partial bold(delta)) |)_(bold(delta) = 0). $
Саме цей $H$ повертає `evaluateError` у GTSAM. Після розв'язку @eq-gn чи @eq-lm оновлення — $X arrow.l X ⊕ bold(delta)$.
#vars(
  [$bold(omega) in RR^3$], [мале обертання (вісь × кут, радіани) у *локальній* системі],
  [$bold(v) in RR^3$], [малий зсув у локальній системі пози (тому в $RR^3$ світу він стає $R bold(v)$)],
  [$bold(xi)$], [локальні координати $"SE"(3)$; *у GTSAM спершу обертання, потім трансляція*],
  [$"Exp"$], [експонента групи: $"Exp"(bold(omega)) = exp([bold(omega)]_times)$ (формула Родріґа)],
  [$H$], [якобіан похибки фактора за локальними координатами змінної],
)

#warn[Порядок $[bold(omega); bold(v)]$ у `Pose3` — найчастіша помилка. Сигми шуму для `Pose3` задаються як `(rot, rot, rot, pos, pos, pos)` — стенсил так і робить. Якщо переплутати, похибка 1 см трактуватиметься як 1 рад.]

== Калібрувальна свобода (gauge)

Якщо всі фактори *відносні* (одометрія між позами, проєкції), то зсув/поворот усього розв'язку як цілого не змінює жодної похибки. Тоді $J^top J$ вироджена (має ядро), і система @eq-gn не має єдиного розв'язку. Цю свободу знімають *абсолютні* фактори: prior на першу позу, GPS/MoCap-вимірювання. У bundle adjustment свобода 7-вимірна: 6 (жорстке перетворення) + 1 (масштаб).

== Шумові моделі та "вибілювання" в GTSAM

GTSAM мінімізує
$ E(X) = 1/2 sum_i norm(bold(e)_i (X))^2_(Sigma_i) = 1/2 sum_i norm(Sigma_i^(-1\/2) bold(e)_i (X))^2, $
тобто кожен залишок множиться на $Sigma_i^(-1\/2)$ ("whitening"), після чого задача стає звичайними НМК. `graph.error(values)` повертає саме $E(X)$ — з множником $1\/2$.
- `noiseModel::Diagonal::Sigmas(σ)` — $Sigma = op("diag")(sigma_1^2, dots)$;
- `noiseModel::Isotropic::Sigma(d, σ)` — $Sigma = sigma^2 I_d$;
- `noiseModel::Gaussian::Covariance(Σ)` — повна коваріація.

Це і є індивідуальний Deliverable 1 "в коді".

// ═════════════════════════════════════════════════════════════════════
= Індивідуальний Deliverable 1 — коваріації в НМК

#deliv[Individual Deliverable 1][10 балів][Цільова функція $f(x) = r(x)^top W r(x)$, $r: RR^n arrow.r RR^m$, $W$ — додатно визначена $m times m$. (1) Звести до стандартних НМК при діагональній $W$; (2) при недіагональній $W$; (3) вивести нормальні рівняння Гаусса–Ньютона для $min r^top W r$.]

== (1) Діагональна $W$

$W = op("diag")(w_1, dots, w_m)$, $w_j > 0$. Тоді
$ f(x) = sum_(j=1)^m w_j r_j (x)^2 = sum_(j=1)^m (sqrt(w_j) r_j (x))^2 = norm(tilde(r)(x))^2, quad tilde(r)(x) = W^(1\/2) r(x), $
де $W^(1\/2) = op("diag")(sqrt(w_1), dots, sqrt(w_m))$. Тобто кожен залишок просто масштабується на $sqrt(w_j)$.
#vars(
  [$r_j (x)$], [$j$-та компонента вектора залишків],
  [$w_j$], [вага $j$-го залишку; в імовірнісній постановці $w_j = 1 \/ sigma_j^2$],
  [$tilde(r)$], ["вибілений" залишок — для нього задача стандартна],
)

== (2) Довільна додатно визначена $W$

Будь-яку симетричну додатно визначену $W$ можна розкласти (Холецький): $W = L L^top$, $L$ — нижньотрикутна з додатною діагоналлю. Тоді
$ f(x) = r^top L L^top r = (L^top r)^top (L^top r) = norm(L^top r(x))^2, quad tilde(r)(x) = L^top r(x). $
Альтернатива — симетричний квадратний корінь $W^(1\/2) = Q Lambda^(1\/2) Q^top$ з власного розкладу $W = Q Lambda Q^top$: $f = norm(W^(1\/2) r)^2$. Обидва варіанти дають ту саму задачу; Холецький дешевший. Якщо $W = Sigma^(-1)$, то $tilde(r) = L^top r$ — це саме "whitening" GTSAM: компоненти $tilde(r)$ некорельовані й мають одиничну дисперсію.
#vars(
  [$L$], [множник Холецького: $W = L L^top$],
  [$Q, Lambda$], [власні вектори (ортогональна матриця) і власні числа $W$],
  [$Sigma$], [коваріація вимірювань; найкраща вага — $W = Sigma^(-1)$],
)

== (3) Нормальні рівняння

Лінеаризація: $r(x + bold(delta)) approx r + J bold(delta)$, $J = partial r \/ partial x |_x$. Підставляємо:
$ f(x + bold(delta)) approx (r + J bold(delta))^top W (r + J bold(delta)) = r^top W r + 2 bold(delta)^top J^top W r + bold(delta)^top J^top W J bold(delta) $
(використано $W = W^top$). Градієнт за $bold(delta)$: $2 J^top W r + 2 J^top W J bold(delta) = 0$, звідки
$ J^top W J bold(delta) = -J^top W r(x). $ <eq-wgn>
$J^top W J$ додатно визначена, якщо $J$ має повний стовпцевий ранг, тож крок єдиний. Перевірка: з $tilde(r) = L^top r$, $tilde(J) = L^top J$ стандартні рівняння $tilde(J)^top tilde(J) bold(delta) = -tilde(J)^top tilde(r)$ збігаються з @eq-wgn, бо $tilde(J)^top tilde(J) = J^top L L^top J = J^top W J$.
#vars(
  [$J$], [якобіан $r$ у поточній точці, $m times n$],
  [$J^top W J$], [зважене наближення гессіана (в GTSAM — "інформаційна матриця" лінеаризованої задачі)],
  [$tilde(J) = L^top J$], [вибілений якобіан],
)

// ═════════════════════════════════════════════════════════════════════
= Індивідуальний Deliverable 2 — групи Лі

#deliv[Individual Deliverable 2][35 балів][Дано $T_i in "SE"(3)$, $i = 1..n$. (1) Замкнена формула для $T^star = op("arg min")_(T in "SE"(3)) 1/n sum_i norm(T - T_i)^2_F$. (2) Зважена версія з $norm(M)^2_Omega = op("tr")(M Omega M^top)$, $Omega_i = op("diag")(omega_i I_3, rho_i)$; вплив ваг. (3) Довести, що це ML-оцінка для $R_i tilde "Langevin"(R, omega_i)$, $bold(t)_i tilde cal(N)(bold(t), rho_i^(-1) I)$. (4) Записати як НМК при $omega_i = 1$, вивести кроки GN і LM. (5, +5) Явні якобіани.]

== (1) Хордальне середнє в $"SE"(3)$

*Розщеплення.* $T - T_i = mat(R - R_i, bold(t) - bold(t)_i; bold(0)^top, 0)$ — нижній рядок зникає. Норма Фробеніуса — сума квадратів усіх елементів, тож
$ norm(T - T_i)^2_F = norm(R - R_i)^2_F + norm(bold(t) - bold(t)_i)^2. $
Задача розпадається на незалежні задачі для $bold(t) in RR^3$ і $R in "SO"(3)$ (множник $1\/n$ на $op("arg min")$ не впливає).

*Трансляція.* $nabla_bold(t) sum_i norm(bold(t) - bold(t)_i)^2 = 2 sum_i (bold(t) - bold(t)_i) = 0$:
$ bold(t)^star = 1/n sum_(i=1)^n bold(t)_i. $

*Обертання.* Розкриваємо: $norm(R - R_i)^2_F = op("tr")(R^top R) - 2 op("tr")(R^top R_i) + op("tr")(R_i^top R_i) = 6 - 2 op("tr")(R^top R_i)$. Отже
$ R^star = op("arg max", limits: #true)_(R in "SO"(3)) op("tr")(R^top M), quad M = sum_(i=1)^n R_i. $
Нехай $M = U S V^top$ (SVD, $S = op("diag")(s_1, s_2, s_3)$, $s_1 >= s_2 >= s_3 >= 0$). Тоді $op("tr")(R^top U S V^top) = op("tr")(V^top R^top U S) = op("tr")(Z S) = sum_k Z_(k k) s_k$, де $Z = V^top R^top U$ ортогональна, отже $Z_(k k) <= 1$. Максимум при $Z = I$, тобто $R = U V^top$. Якщо $det(U V^top) = -1$, це відбиття; найкраще обертання тоді отримуємо, "жертвуючи" найменшим сингулярним числом:
$ R^star = U op("diag")(1, 1, det(U V^top)) V^top = cal(P)_("SO"(3)) (sum_i R_i). $
Тобто *хордальне середнє обертань = проєкція арифметичного середнього матриць на $"SO"(3)$*.
#vars(
  [$T^star, R^star, bold(t)^star$], [шукане середнє та його частини],
  [$M$], [сума матриць обертання (сама вона, загалом, *не* обертання)],
  [$U, S, V$], [SVD матриці $M$],
  [$Z$], [допоміжна ортогональна матриця в доведенні],
  [$cal(P)_("SO"(3))$], [проєкція на $"SO"(3)$ — найближче за Фробеніусом обертання],
)

== (2) Зважене хордальне середнє

$M = T - T_i = mat(Delta R, Delta bold(t); bold(0)^top, 0)$, $Omega_i = mat(omega_i I_3, bold(0); bold(0)^top, rho_i)$. Тоді
$ M Omega_i M^top = mat(omega_i Delta R Delta R^top + rho_i Delta bold(t) Delta bold(t)^top, bold(0); bold(0)^top, 0), quad op("tr")(M Omega_i M^top) = omega_i norm(R - R_i)^2_F + rho_i norm(bold(t) - bold(t)_i)^2. $
Задача знову розпадається, і ті самі міркування дають
$ bold(t)^star = (sum_i rho_i bold(t)_i) / (sum_i rho_i), quad R^star = cal(P)_("SO"(3)) (sum_i omega_i R_i). $

*Вплив ваг.* $omega_i$ впливає *лише* на обертання, $rho_i$ — *лише* на трансляцію. Якщо для одного $i$ обидва коефіцієнти дуже великі ($omega_i, rho_i arrow.r infinity$), то $sum_j omega_j R_j approx omega_i R_i$ і $bold(t)^star arrow.r bold(t)_i$: середнє "прилипає" до $T_i$ (це вимірювання вважається майже точним). Якщо велике лише $omega_i$ — до $R_i$ прилипає тільки обертання, а трансляція лишається зваженим середнім інших.
#vars(
  [$omega_i$], [вага (концентрація) обертальної частини $i$-го вимірювання],
  [$rho_i$ (у завданні $rho.alt_i$)], [вага трансляційної частини, обернена дисперсія],
  [$Delta R, Delta bold(t)$], [$R - R_i$ та $bold(t) - bold(t)_i$],
)

== (3) Це ML-оцінка (Langevin + Gauss)

Розподіл Ланжевена (фон Мізеса–Фішера для матриць) з модою $R$ і концентрацією $omega$:
$ p(R_i | R) = 1 / (c(omega_i)) exp(omega_i op("tr")(R^top R_i)). $
Гаусів розподіл трансляції з коваріацією $rho_i^(-1) I$:
$ p(bold(t)_i | bold(t)) = (rho_i / (2 pi))^(3\/2) exp(-rho_i / 2 norm(bold(t)_i - bold(t))^2). $
Вимірювання незалежні, і $R_i$ не залежить від $bold(t)$, тож
$ -log product_i p(T_i | T) = sum_i [ -omega_i op("tr")(R^top R_i) + rho_i / 2 norm(bold(t) - bold(t)_i)^2 ] + C, $
де $C$ не залежить від $T$ (нормувальні константи $c(omega_i)$ залежать лише від $omega_i$). Використаємо тотожність з пункту (1): $op("tr")(R^top R_i) = 3 - 1/2 norm(R - R_i)^2_F$. Тоді
$ -log product_i p(T_i | T) = 1/2 sum_i [omega_i norm(R - R_i)^2_F + rho_i norm(bold(t) - bold(t)_i)^2] + C' = 1/2 sum_i norm(T - T_i)^2_(Omega_i) + C'. $
Максимізація правдоподібності ⇔ мінімізація $-log$ ⇔ мінімізація $1/n sum_i norm(T - T_i)^2_(Omega_i)$ (додатні множники $1\/2$, $1\/n$ і константа $C'$ не змінюють $op("arg min")$). $square$
#vars(
  [$c(omega)$], [нормувальна константа Ланжевена (інтеграл по $"SO"(3)$); не залежить від $R$],
  [$C, C'$], [константи, що не залежать від $T$],
)
#note[Якщо у вашому конспекті Ланжевен записано як $exp(omega\/2 dot op("tr")(R^top R_i))$, доведення те саме, лише в $Omega_i$ з'явиться $omega_i \/ 2$ замість $omega_i$ — на форму розв'язку це не впливає.]

== (4) НМК і кроки Гаусса–Ньютона / Левенберга–Марквардта ($omega_i = 1$)

*Залишки.* Для кожного вимірювання — 12-вимірний вектор
$ r_i (T) = mat(op("vec")(R - R_i); sqrt(rho_i) (bold(t) - bold(t)_i)) in RR^(12), quad min_T sum_i norm(r_i (T))^2. $
*Параметризація кроку* — ретракція @eq-retract, $bold(xi) = (bold(omega), bold(v))$. З @eq-pose-first-order:
$ r_i (T ⊕ bold(xi)) approx mat(op("vec")(R - R_i + R [bold(omega)]_times); sqrt(rho_i) (bold(t) - bold(t)_i + R bold(v))) = r_i (T) + J_i bold(xi). $
Оскільки $[bold(omega)]_times = sum_k omega_k [bold(e)_k]_times$ (лінійно по $bold(omega)$),
$ J_i = mat(J_R, 0_(9 times 3); 0_(3 times 3), sqrt(rho_i) R), quad J_R = mat(op("vec")(R [bold(e)_1]_times), op("vec")(R [bold(e)_2]_times), op("vec")(R [bold(e)_3]_times)) in RR^(9 times 3). $ <eq-jac>

*Крок Гаусса–Ньютона.* $(sum_i J_i^top J_i) bold(xi) = -sum_i J_i^top r_i$. Блоки обчислюються явно:
- $J_R^top J_R = 2 I_3$: стовпці ортогональні, $norm(R [bold(e)_k]_times)^2_F = norm([bold(e)_k]_times)^2_F = 2$;
- $J_R^top op("vec")(R) = 0$: $chevron.l R [bold(e)_k]_times, R chevron.r = op("tr")([bold(e)_k]_times^top) = 0$;
- $J_R^top op("vec")(R_i) = bold(w)(A_i)$, де $A_i = R^top R_i$ і $bold(w)(A) = (A_32 - A_23, med A_13 - A_31, med A_21 - A_12)^top = 2 op("vee")(op("skew")(A))$, $op("skew")(A) = (A - A^top) \/ 2$. (Покомпонентно: $chevron.l R [bold(e)_k]_times, R_i chevron.r = op("tr")([bold(e)_k]_times^top R^top R_i)$, а $[bold(e)_1]_times$ має $+1$ у позиції $(3, 2)$ і $-1$ у $(2, 3)$ — звідси $A_32 - A_23$.)

Звідси кроки (обертання й трансляція незалежні — матриця блочно-діагональна):
$ bold(omega)_"GN" = 1/(2n) sum_i bold(w)(R^top R_i) = 1/n sum_i op("vee")(op("skew")(R^top R_i)), quad bold(v)_"GN" = R^top (overline(bold(t))_rho - bold(t)), quad overline(bold(t))_rho = (sum_i rho_i bold(t)_i) / (sum_i rho_i). $
Оновлення: $R arrow.l R "Exp"(bold(omega)_"GN")$, $bold(t) arrow.l bold(t) + R bold(v)_"GN" = overline(bold(t))_rho$.

*Інтуїція.* Для малого кута $op("vee")(op("skew")("Exp"(bold(theta)))) = sin norm(bold(theta)) dot bold(theta) \/ norm(bold(theta)) approx bold(theta)$, тож $bold(omega)_"GN" approx 1/n sum_i "Log"(R^top R_i)$ — середнє "відхилень" вимірювань від поточної оцінки в дотичному просторі. Трансляційна частина лінійна, тому GN знаходить її точно за *один* крок.

*Крок Левенберга–Марквардта.* $(H + lambda D) bold(xi) = -bold(g)$, де $H = op("diag")(2 n I_3, (sum_i rho_i) I_3)$, $bold(g) = sum_i J_i^top r_i$. З $D = op("diag")(H)$ (Марквардт):
$ bold(omega)_"LM" = 1/(1 + lambda) bold(omega)_"GN", quad bold(v)_"LM" = 1/(1 + lambda) bold(v)_"GN". $
З $D = I$: $bold(omega)_"LM" = 2n \/ (2n + lambda) dot bold(omega)_"GN"$, $bold(v)_"LM" = (sum rho_i) \/ (sum rho_i + lambda) dot bold(v)_"GN"$. Правило: якщо $sum norm(r_i)^2$ зменшилась — приймаємо крок і $lambda arrow.l lambda \/ 10$, інакше відкидаємо і $lambda arrow.l 10 lambda$.
#vars(
  [$r_i (T)$], [залишок $i$-го вимірювання: 9 компонент обертання + 3 трансляції],
  [$J_i$], [якобіан $r_i$ за $bold(xi)$ у точці $bold(xi) = 0$, $12 times 6$],
  [$J_R$], [якобіан $op("vec")(R "Exp"(bold(omega)))$ за $bold(omega)$],
  [$A_i = R^top R_i$], [відносне обертання вимірювання щодо поточної оцінки],
  [$bold(w)(A)$, $op("vee")$, $op("skew")$], [вектор з антисиметричної частини $A$; $op("vee")$ — обернена до $[dot]_times$],
  [$bold(g)$, $H$], [градієнт (без множника 2) і наближений гессіан],
)

== (5) Явні якобіани (+5)

Позначимо стовпці $R = [bold(r)_1 | bold(r)_2 | bold(r)_3]$. Стовпці $[bold(e)_k]_times$: для $k = 1$ — $(bold(0), bold(e)_3, -bold(e)_2)$, для $k = 2$ — $(-bold(e)_3, bold(0), bold(e)_1)$, для $k = 3$ — $(bold(e)_2, -bold(e)_1, bold(0))$. Множимо на $R$ зліва:
$ R [bold(e)_1]_times = [bold(0) | bold(r)_3 | -bold(r)_2], quad R [bold(e)_2]_times = [-bold(r)_3 | bold(0) | bold(r)_1], quad R [bold(e)_3]_times = [bold(r)_2 | -bold(r)_1 | bold(0)]. $
Отже (vec — по стовпцях, блоки $3 times 1$):
$ J_R = mat(bold(0), -bold(r)_3, bold(r)_2; bold(r)_3, bold(0), -bold(r)_1; -bold(r)_2, bold(r)_1, bold(0)), quad partial (sqrt(rho_i) (bold(t) ⊕ bold(v) - bold(t)_i)) / (partial bold(v)) = sqrt(rho_i) R, quad partial / (partial bold(omega)) (dots) = 0. $
Цей самий $J_R$ реалізовано в командному Deliverable 5 і перевірено числово (результат GTSAM збігся з замкненою формулою до $10^(-12)$ рад).

// ═════════════════════════════════════════════════════════════════════
= Підготовка середовища

```bash
bash ros2-docker/run_lab_dev.sh lab7 --build-only   # збирання lab_7 у Docker
bash ros2-docker/run_lab_dev.sh lab7                # + запуск Deliverable 2/3 з RViz2
```
Всередині контейнера:
```bash
source /opt/ros/humble/setup.bash && cd /workspace
colcon build --symlink-install --packages-select lab_7 && source install/setup.bash
ros2 run lab_7 deliverable_1
ros2 launch lab_7 deliverable_2_3.launch.yaml max_solver_iterations:=3 use_mocap:=false
ros2 launch lab_7 deliverable_4.launch.yaml
# macOS: додайте до launch-команд  rviz:=false foxglove:=true
ros2 run lab_7 deliverable_5
```

*Версія GTSAM* в образі — `4.2a9` (`ros-humble-gtsam`). Це важливо: у ній `evaluateError` ще приймає `boost::optional<Matrix&> H`, як у стенсилі. У GTSAM 4.3+ сигнатура інша (`OptionalMatrixType H`), і код з інтернету під новішу версію тут не збереться.

#warn[У поточній версії `include/deliverable_2_3.h` у репозиторії конструктор передає в базовий клас `poseclKey` — такої змінної немає, тож `lab_7` не збереться, доки там не буде `poseKey` (як у коді нижче).]

*Візуалізація на macOS — Foxglove замість RViz2.* RViz2 (Ogre 1.12) не може створити OpenGL-вікно через XQuartz ні з програмним рендерингом, ні з непрямим GLX (`OpenGL 1.5 is not supported` → `Unable to create the rendering window`). Тому обидва launch-файли мають аргументи `rviz:=true|false` і `foxglove:=true|false`, а `run_lab_dev.sh lab7` на Mac публікує порт 8765 і за замовчуванням запускає `rviz:=false foxglove:=true`.
+ Відкрийте #link("https://app.foxglove.dev")[app.foxglove.dev] → *Open connection* → *Foxglove WebSocket* → `ws://localhost:8765`.
+ Імпортуйте готовий layout: *Layouts → Import from file* → `VNAV-labs/lab7/config/lab7_foxglove.json` (3D-панель, кадр `world`, лінії траєкторій D2/D3 і точки/камери D4 уже ввімкнені). Або додайте панель *3D* вручну й увімкніть топіки з таблиці нижче.
+ Скриншот — кнопкою в панелі або `Cmd+Shift+4`.

На Linux RViz2 працює як звичайно (`rviz:=true`, за замовчуванням). Для Deliverable 4 launch відкриває `rviz/deliverable_4.rviz` (у стенсилі конфіги D2/3 і D4 були переплутані — виправлено).

#table(
  columns: (auto, 1fr),
  inset: 5pt, stroke: 0.4pt + luma(200),
  fill: (_, y) => if y == 0 { luma(235) },
  [*Deliverable*], [*Топіки для панелі 3D*],
  [D2/D3], [`/gt_trajectory(_lines)` — еталон, `/trajectory(_lines)` — зашумлена, `/initial_trajectory(_lines)` — початкова, `/optimal_trajectory(_lines)` — оптимізована, `/robot_pose`],
  [D4], [`/landmarks_gt`, `/landmarks_init`, `/landmarks_optimal`, `/poses_gt`, `/poses_init`, `/poses_optimal`],
)

// ═════════════════════════════════════════════════════════════════════
= Командний Deliverable 1 — вступ до GTSAM

#deliv[Team Deliverable 1][10 балів][За hands-on гайдом GTSAM (до розділу 3 включно): 1a — odometry `BetweenFactor`; 1b — "GPS" `UnaryFactor`; 1c — навмисно неточне початкове наближення. Зберегти значення до/після оптимізації в `deliverable_1.txt`.]

== Теорія

Змінні — три пози `Pose2` $x_k = (x, y, theta)$. Фактори:
- *Prior* на $x_1$: $bold(e) = "Log"(mu^(-1) x_1)$ — "поза $x_1$ близька до $mu$".
- *BetweenFactor* (одометрія): $bold(e) = "Log"(z_(k,k+1)^(-1) dot (x_k^(-1) x_(k+1)))$ — виміряне відносне переміщення $z$ порівнюється з передбаченим $x_k^(-1) x_(k+1)$.
- *UnaryFactor* ("GPS"): $bold(e) = (x - m_x, med y - m_y)$ — вимірює лише позицію.

*Якобіан UnaryFactor* (уже написаний у стенсилі — варто розібратися чому). Ретракція `Pose2`: $(x, y, theta) ⊕ (delta_x, delta_y, delta_theta)$ зсуває позицію на $R(theta) (delta_x, delta_y)^top$ (зсув у *локальній* системі робота). Отже
$ H = partial bold(e) \/ partial bold(delta) = mat(cos theta, -sin theta, 0; sin theta, cos theta, 0) = [R(theta) | bold(0)]. $
#vars(
  [$mu$], [середнє prior-фактора (`priorMean` $= (0, 0, 0)$)],
  [$z_(k,k+1)$], [виміряна одометрія між $x_k$ та $x_(k+1)$ (тут $(2, 0, 0)$ — "2 м вперед")],
  [$(m_x, m_y)$], [виміряна "GPS"-позиція],
  [$R(theta)$], [матриця повороту $2 times 2$ на кут курсу робота],
)

== Код

```cpp
  // Start of 1a.
  graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
  graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));
  // End of 1a.
```
```cpp
  // Start of 1b.
  graph.add(UnaryFactor(1, 0.0, 0.0, unaryNoise));
  graph.add(UnaryFactor(2, 2.0, 0.0, unaryNoise));
  graph.add(UnaryFactor(3, 4.0, 0.0, unaryNoise));
  // End of 1b.
```
```cpp
  // Start of 1c.
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  // End of 1c.
```
#codevars(
  ([`graph`], [`NonlinearFactorGraph`], [контейнер факторів — наша сума @eq-map]),
  ([`1, 2, 3`], [`Key`], [ключі змінних (поз); у GTSAM це просто цілі числа]),
  ([`odometry`], [`Pose2`], [виміряне відносне переміщення]),
  ([`odometryNoise`, `unaryNoise`], [`SharedNoiseModel`], [сигми шуму: $(0.2, 0.2, 0.1)$ і $(0.1, 0.1)$]),
  ([`initial`], [`Values`], [початкове наближення (лінеаризація стартує звідси)]),
)

== Результат (реальний прогін)

Збережіть вивід: `ros2 run lab_7 deliverable_1 > deliverable_1.txt`.
```text
Initial Estimate            Optimized Estimate
Value 1: (0.5, 0, 0.2)      Value 1: (1.4e-12, -3.9e-12, -4.3e-12)
Value 2: (2.3, 0.1, -0.2)   Value 2: (2, 6.6e-12, 1.9e-13)
Value 3: (4.1, 0.1, 0.1)    Value 3: (4, -2.2e-12, 1.9e-13)
```
Оптимум — рівно $(0, 0, 0), (2, 0, 0), (4, 0, 0)$: усі вимірювання взаємно узгоджені (одометрія "2 м вперед" і GPS $0 arrow.r 2 arrow.r 4$), тож існує розв'язок з нульовою похибкою. Залишки $~10^(-12)$ — машинна точність.

// ═════════════════════════════════════════════════════════════════════
= Командний Deliverable 2 — 3D pose graph

#deliv[Team Deliverable 2][10 балів][У `deliverable_2_3.cpp`: 2a — `BetweenFactor<Pose3>` для всіх вимірювань одометрії; 2b — prior на вузол 1. Скриншоти RViz після 1, 3 і 6 ітерацій Гаусса–Ньютона.]

== Що генерує стенсил

- Еталонна траєкторія — спіраль: $N = 500$ поз, радіус 2 м, два оберти ($4 pi$), підйом від 0 до 1 м; перша поза $(2, 0, 0)$.
- Одометрія: $z_i = T_i^(-1) T_(i+1)$ з шумом $sigma_"rot" = sigma_"pos" = 10^(-2)$ (у стенсилі це додається в *локальних* координатах через `Pose3::Expmap`).
- Початкове наближення — *інша* спіраль ($3 pi$, еліпс $1.2 times 0.5$, підйом до 2 м), навмисно далека.

== Код

```cpp
    // Start of 2a
    // measurements[i] is the relative pose between gt poses i and i+1, which
    // are variables with keys i+1 and i+2 (keys start at 1).
    for (size_t i = 0; i < measurements.size(); ++i) {
      graph.add(BetweenFactor<Pose3>(i + 1, i + 2, measurements[i], odometryNoise));
    }
    // End 2a.
```
```cpp
      // Start of 2b.
      graph.add(PriorFactor<Pose3>(1, initial_pose, initialNoise));
      // End 2b.
```
#codevars(
  ([`measurements`], [`vector<Pose3>`], [499 зашумлених відносних поз $z_i$]),
  ([`i + 1`, `i + 2`], [`Key`], [ключі сусідніх поз; нумерація змінних починається з 1, а вимірювань — з 0]),
  ([`odometryNoise`], [`Diagonal`], [сигми в порядку `Pose3`: 3 обертальні, 3 позиційні]),
  ([`initial_pose`], [`Pose3`], [$(I, (2, 0, 0))$ — збігається з першою еталонною позою]),
  ([`max_solver_iterations_`], [`size_t`], [параметр ROS — скільки ітерацій GN дозволено]),
)

== Результати (реальні прогони, без MoCap)

#table(
  columns: (auto, auto, auto, auto, auto, auto),
  inset: 5pt, stroke: 0.4pt + luma(200), align: center,
  fill: (_, y) => if y == 0 { luma(235) },
  [*Ітерація*], [0 (старт)], [1], [2], [3], [5],
  [$E(X)$], [$approx 7.8 dot 10^3$], [$approx 2.5 dot 10^3$], [$approx 110$], [$approx 10^(-4)$], [$3 dot 10^(-25)$ (збіглося)],
)
(Точні числа трохи різняться між запусками — шум генерується випадково.)

*Що маєте побачити в RViz* (`lab7.rviz`; числа нижче перевірені прогоном, картинку — ні, прогін був без GUI): зелена — еталон, біло-червона — зашумлена траєкторія, червона — початкове наближення, бірюзова — оптимізована.
- Після 1 ітерації бірюзова вже схожа на спіраль, але спотворена: задача сильно нелінійна через обертання ($theta$ старту далекий від правильного на сотні градусів у кінці траєкторії).
- Після 3 ітерацій — практично збіглося.
- Після 6 — ідеальний збіг… *із зашумленою* траєкторією, а не з еталоном!

#note[*Чому оптимум = зашумлена траєкторія.* Граф — ланцюжок (дерево): 499 відносних факторів + 1 prior на 500 змінних. Для дерева завжди існує розв'язок, що задовольняє всі фактори точно (просто "проінтегрувати" одометрію від prior), тож $E^star = 0$ і оптимум — це dead reckoning. Без додаткової інформації (loop closure, GPS/MoCap) оптимізація не може прибрати накопичений дрейф. Це і мотивує Deliverable 3.]

// ═════════════════════════════════════════════════════════════════════
= Командний Deliverable 3 — фактор Motion Capture

#deliv[Team Deliverable 3][15 балів][3a — реалізувати `MoCapPosition3Factor` (вимірювання — `Point3` позиції, змінна — `Pose3`) з якобіаном; 3b — шумова модель і додавання факторів. Скриншоти після 1, 3, 6 ітерацій з `use_mocap:=true`.]

== Виведення фактора

Модель вимірювання: MoCap бачить *позицію* робота, $h(T) = bold(t)$. Похибка:
$ bold(e)(T) = bold(t) - bold(m) in RR^3. $
Якобіан за локальними координатами `Pose3` $bold(xi) = (bold(omega), bold(v))$: з @eq-pose-first-order позиція після ретракції — $bold(t) + R bold(v)$ (обертання $bold(omega)$ позицію в першому порядку не змінює). Отже
$ H = (partial bold(e)(T ⊕ bold(xi))) / (partial bold(xi)) = [0_(3 times 3) | R] in RR^(3 times 6). $
Це той самий `UnaryFactor` з T-1, лише в 3D: там було $[R(theta) | bold(0)]$, бо в `Pose2` порядок координат $(x, y, theta)$ — позиція *спочатку*. У `Pose3` порядок навпаки, тому нулі зліва.
#vars(
  [$bold(m)$], [виміряна MoCap-позиція (`m_`)],
  [$bold(t), R$], [позиція й обертання поточної оцінки пози `p`],
  [$H$], [$3 times 6$: нулі за обертанням, $R$ за локальним зсувом],
)

== Код 3a — `include/deliverable_2_3.h`

```cpp
  MoCapPosition3Factor(gtsam::Key poseKey,
                       const gtsam::Point3& m,
                       const gtsam::SharedNoiseModel& model)
      : NoiseModelFactor1<Pose3>(model, poseKey), m_(m) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p,
                              boost::optional<gtsam::Matrix&> H = boost::none) const {
    // 3a. Complete definition of factor
    // Measurement model h(T) = t (position of the pose). With GTSAM's Pose3
    // retraction T * Exp([w; v]) the position moves by R * v to first order,
    // so dh/d[w; v] = [0_{3x3}, R].
    if (H) {
      gtsam::Matrix36 J;
      J << gtsam::Matrix3::Zero(), p.rotation().matrix();
      *H = J;
    }
    return p.translation() - m_;
    // End 3a.
  }
```
#tip[Той самий якобіан GTSAM дає вбудовано: `p.translation(H)` заповнює `OptionalJacobian<3,6>` рівно $[0 | R]$. Але для звіту варто написати явно — оцінюють саме виведення.]

== Код 3b — `src/deliverable_2_3.cpp`

```cpp
      // Start of 3b.
      // MoCap measures a 3D position, so the noise model is 3-dimensional.
      const noiseModel::Diagonal::shared_ptr mocapNoise =
          noiseModel::Diagonal::Sigmas(Vector3::Constant(mocap_std_dev));

      //  TODO: add the MoCap factors
      // ...
      for (const auto& [key, position] : mocap_measurements) {
        graph.add(MoCapPosition3Factor(key, position, mocapNoise));
      }
      // End 3b.
```
#codevars(
  ([`mocap_measurements`], [`vector<pair<int, Point3>>`], [пари (ключ пози, виміряна позиція); 10 вимірювань — ключі 1, 51, …, 451]),
  ([`mocap_std_dev`], [`double`], [$10^(-10)$ м — MoCap майже ідеальний]),
  ([`mocapNoise`], [`Diagonal` (3D)], [три однакові сигми — по одній на $x, y, z$]),
  ([`key`, `position`], [`int`, `Point3`], [structured binding з пари (C++17)]),
)
*Prior не потрібен*: 10 абсолютних позицій (неколінеарних) фіксують і положення, і орієнтацію всієї траєкторії — gauge знято. (У стенсилі prior додається лише при `use_mocap:=false`.)

== Результати (реальні прогони, з MoCap)

#table(
  columns: (auto, auto, auto, auto, auto, auto, auto),
  inset: 5pt, stroke: 0.4pt + luma(200), align: center,
  fill: (_, y) => if y == 0 { luma(235) },
  [*Ітерація*], [0], [1], [2], [3], [5], [6],
  [$E(X)$], [$2.3 dot 10^(21)$], [$1.6 dot 10^(21)$], [$3.8 dot 10^(20)$], [$5.5 dot 10^(19)$], [$1.5 dot 10^(14)$], [$2.3 dot 10^7$],
)
Числа здаються величезними, бо $sigma = 10^(-10)$ м дає вагу $1\/sigma^2 = 10^(20)$. Переведімо в метри: $E = 2.3 dot 10^7$ ⇒ сумарний квадрат відхилень $2 E sigma^2 approx 5 dot 10^(-13)$ м², тобто позиції в MoCap-точках збігаються з вимірюваннями до мікронів. Отже в RViz бірюзова траєкторія після 3–6 ітерацій має лягти на *зелену* (еталон), а не на зашумлену: MoCap прибирає дрейф.

// ═════════════════════════════════════════════════════════════════════
= Командний Deliverable 4 — bundle adjustment

#deliv[Team Deliverable 4][20 балів][Розв'язати BA в `deliverable_4.cpp`: prior на $x_0$, `GenericProjectionFactor` для кожного спостереження, prior на $l_0$ (масштаб). Звітувати початкові та фінальні значення; скриншот RViz (landmarks і камери до/після + еталон).]

== Теорія

Змінні: пози камер $x_i = T^W_(C_i) in "SE"(3)$ (камера → світ, як у GTSAM) і точки $l_j in RR^3$. Спостереження — піксель $bold(u)_(i j)$ точки $j$ у камері $i$. Модель вимірювання (камера-обскура):
$ bold(p)_C = R_i^top (l_j - bold(t)_i), quad pi(x_i, l_j) = mat(f_x p_x \/ p_z + c_x; f_y p_y \/ p_z + c_y), quad bold(e)_(i j) = pi(x_i, l_j) - bold(u)_(i j). $
BA мінімізує сумарну *похибку перепроєкції*:
$ min_({x_i}, {l_j}) sum_(i,j) norm(pi(x_i, l_j) - bold(u)_(i j))^2_(Sigma_u), quad Sigma_u = sigma_u^2 I_2. $
Це ML-оцінка для гаусового піксельного шуму — кращого способу оцінити пози й точки за самими спостереженнями немає (на відміну від 5/8-точкових методів Lab 6, які мінімізують алгебраїчну похибку).

*Gauge.* Усі проєкційні фактори інваріантні до перетворення подібності світу ($R, bold(t)$ і масштаб $s$ — 7 ступенів свободи). Prior на $x_0$ фіксує 6, prior на $l_0$ — масштаб (відстань від камери 0 до точки 0) і ще 2 ступені, які й так зафіксовані — легке надвизначення, не шкодить.
#vars(
  [$x_i = (R_i, bold(t)_i)$], [поза $i$-ї камери у світі (Symbol `'x'`, i)],
  [$l_j$], [3D-точка (landmark) у світі (Symbol `'l'`, j)],
  [$bold(p)_C = (p_x, p_y, p_z)$], [точка в системі камери],
  [$f_x, f_y, c_x, c_y$], [intrinsics `Cal3_S2(50, 50, 0, 50, 50)`: фокус 50 px, центр (50, 50), skew 0],
  [$bold(u)_(i j)$], [виміряний піксель (`kpt_coords_`)],
  [$sigma_u$], [піксельний шум, 1 px (`measurementNoise`)],
)

== Код

```cpp
    // TODO: Add a prior on pose x0. This indirectly specifies where the origin is.
    // 0.3 rad on roll, pitch, yaw and 0.1 m on x, y, z
    noiseModel::Diagonal::shared_ptr poseNoise =
        noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),
                                      Vector3::Constant(0.1)).finished());
    graph.add(PriorFactor<Pose3>(Symbol('x', 0), camera_poses[0], poseNoise));
```
```cpp
    // TODO: Add GenericProjectionFactor factors to the graph.
    for (const auto& [cam_idx, kpt] : measurements) {
      graph.add(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
          kpt.kpt_coords_, measurementNoise, Symbol('x', cam_idx),
          Symbol('l', kpt.lmk_idx_), K));
    }
```
```cpp
    // TODO: ... add a prior on the position of the first landmark (fixes scale).
    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.add(PriorFactor<Point3>(Symbol('l', 0), landmarks[0], pointNoise));
```
#codevars(
  ([`camera_poses`], [`vector<Pose3>`], [8 еталонних камер на колі радіуса 3 м, усі дивляться в центр куба]),
  ([`landmarks`], [`vector<Point3>`], [8 вершин куба $plus.minus 1$ м]),
  ([`measurements`], [`vector<pair<size_t, Keypoint>>`], [64 спостереження: (індекс камери, (індекс точки, піксель))]),
  ([`cam_idx`, `kpt`], [`size_t`, `Keypoint`], [розпакована пара]),
  ([`Symbol('x', i)`], [`Key`], [ключ з літерою: у 64-бітному ключі старший байт — символ, решта — номер]),
  ([`K`], [`Cal3_S2::shared_ptr`], [калібрування камери]),
  ([`poseNoise`, `pointNoise`], [`Diagonal`, `Isotropic`], [сигми prior-факторів ("будь-які розумні" за завданням)]),
)

== Результати (реальний прогін)

```text
initial error = 12033.96
LM iterations: 604.2 → 8.31 → 3.7e-3 → 4.7e-10 → 1.6e-23
final error   = 1.56e-23
l1 covariance: diag ≈ (0.041, 0.098, 0.097)
l2 covariance: diag ≈ (0.111, 0.104, 0.566)
```
- Похибка падає до машинного нуля: спостереження згенеровані *без шуму*, тож еталон — точний розв'язок, і LM його знаходить (5 ітерацій).
- Маргінальні коваріації точок кажуть, наскільки кожну точку "визначено" графом. У $l_2$ дисперсія по $z$ помітно більша ($0.57$) — ймовірно тому, що всі камери лежать у площині $z = 0$ і вертикальну координату точок визначено гірше.

*RViz / Foxglove* (розділ 5): зелені — еталон, червоні — початкове наближення (точки зсунуті на $(-0.25, 0.2, 0.15)$, камери повернуті Родріґом $(-0.1, 0.2, 0.25)$), сині — після оптимізації, мають лягти на зелені.

// ═════════════════════════════════════════════════════════════════════
= Командний Deliverable 5 (опційний) — усереднення $"SO"(3)$

#deliv[Team Deliverable 5][+15 балів][Реалізувати фактор, що кодує хордальну похибку $norm(R - R_i)_F$ між оцінкою й виміряним обертанням, побудувати граф, задати грубе початкове наближення, оптимізувати, порівняти.]

== Теорія

Це індивідуальний Deliverable 2 (пункти 1, 4, 5) для $"SO"(3)$: залишок $bold(e)_i (R) = op("vec")(R - R_i) in RR^9$, якобіан — $J_R$ з @eq-jac. Сума $sum_i norm(bold(e)_i)^2$ — хордальна функція втрат, оптимум — $cal(P)_("SO"(3))(sum R_i)$. Стенсил задає шум 9-вимірний з $sigma = 10^(-2)$ — розмірність похибки має збігатися з розмірністю шумової моделі.

== Код — `include/deliverable_5.h`

```cpp
// Unary factor with the 9-dimensional error vec(R - R_meas) (column-major), so
// that the squared error is the chordal distance ||R - R_meas||_F^2.
class FrobeniusNormFactor : public NoiseModelFactor1<Rot3> {
 private:
  Rot3 measured_;

 public:
  FrobeniusNormFactor(Key key, const Rot3& measured, const SharedNoiseModel& model)
      : NoiseModelFactor1<Rot3>(model, key), measured_(measured) {}

  Vector evaluateError(const Rot3& R, boost::optional<Matrix&> H = boost::none) const override {
    const Matrix3 Rm = R.matrix();
    if (H) {
      // Rot3 retraction: R * Exp(w) ~ R (I + [w]x), so d vec(R Exp(w)) / dw_k
      // = vec(R [e_k]x). Columns of R [e_k]x: [0, r3, -r2], [-r3, 0, r1], [r2, -r1, 0].
      const Vector3 r1 = Rm.col(0), r2 = Rm.col(1), r3 = Rm.col(2);
      Matrix J = Matrix::Zero(9, 3);
      J.block<3, 1>(3, 0) = r3;
      J.block<3, 1>(6, 0) = -r2;
      J.block<3, 1>(0, 1) = -r3;
      J.block<3, 1>(6, 1) = r1;
      J.block<3, 1>(0, 2) = r2;
      J.block<3, 1>(3, 2) = -r1;
      *H = J;
    }
    const Matrix3 D = Rm - measured_.matrix();
    return Eigen::Map<const Vector9>(D.data());
  }
};
```
#note[Eigen зберігає матриці по стовпцях, тож `Eigen::Map<const Vector9>(D.data())` — це саме $op("vec")(D)$ по стовпцях, узгоджений з рядками якобіана: рядки 0–2 — перший стовпець $R$, 3–5 — другий, 6–8 — третій.]

== Код — `src/deliverable_5.cpp`

```cpp
  NonlinearFactorGraph graph;
  // ... (шумова модель noise вже є в стенсилі)
  for (const Rot3& R_i : measurements) {
    graph.add(FrobeniusNormFactor(1, R_i, noise));
  }
  graph.print("Factor graph:\n");
  // ... (initial вже є в стенсилі: вимірювання 0, повернуте на 45° навколо y)
  initial.print("Initial estimate:\n");

  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  const Values result = LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  result.print("Final result:\n");
  std::cout << "initial error = " << graph.error(initial) << std::endl;
  std::cout << "final error = " << graph.error(result) << std::endl;

  // Closed form for comparison: projection of the mean onto SO(3).
  Matrix3 M = Matrix3::Zero();
  for (const Rot3& R_i : measurements) M += R_i.matrix();
  Eigen::JacobiSVD<Matrix3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3 S = Matrix3::Identity();
  S(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  const Rot3 closed_form(svd.matrixU() * S * svd.matrixV().transpose());
  std::cout << "angle(GTSAM, closed form) = "
            << result.at<Rot3>(1).between(closed_form).axisAngle().second << " rad\n";
```
#codevars(
  ([`measurements`], [`vector<Rot3>`], [10 зашумлених обертань навколо $I$ ($sigma = 10^(-2)$ рад)]),
  ([`noise`], [`Diagonal` (9D)], [по одній сигмі на кожен елемент матриці]),
  ([`1`], [`Key`], [єдина змінна — шукане середнє обертання]),
  ([`closed_form`], [`Rot3`], [$cal(P)_("SO"(3))(sum R_i)$ для перевірки]),
)

== Результати (реальний прогін)

```text
Initial estimate: R ≈ Ry(45°) · R_0      initial error = 57463.7
Final result:     R ≈ I (відхилення ~0.4°) final error   = 31.59
LM: 4 ітерації
angle(GTSAM, closed form) = 8.8e-13 rad
```
Навіть зі старту за $45°$ LM сходиться за 4 ітерації до *точно* того самого обертання, що й замкнена формула (розбіжність $10^(-12)$ рад — машинна точність). Фінальна похибка не нульова — це залишковий розкид самих вимірювань навколо їх середнього.

// ═════════════════════════════════════════════════════════════════════
= Чекліст здачі

- *Індивідуально (Gradescope, PDF):* I-1 — пункти (1)–(3) з розділу 3; I-2 — (1)–(4) з розділу 4, бонус (5) — явний $J_R$.
- *Команда (папка `lab7` у репозиторії):* увесь пакет з кодом + PDF з:
  - T-1: `deliverable_1.txt` (початкові й фінальні значення 3 поз);
  - T-2: 3 скриншоти RViz (1, 3, 6 ітерацій, `use_mocap:=false`);
  - T-3: 3 скриншоти RViz (1, 3, 6 ітерацій, `use_mocap:=true`);
  - T-4: початкові й фінальні значення точок і поз + скриншот RViz;
  - T-5 (опц.): початкове й фінальне обертання, похибки.

*Корисні команди для збору виводу:*
```bash
ros2 run lab_7 deliverable_1 > deliverable_1.txt
# (macOS: додайте rviz:=false foxglove:=true і дивіться в Foxglove)
ros2 launch lab_7 deliverable_2_3.launch.yaml max_solver_iterations:=1 use_mocap:=false
ros2 launch lab_7 deliverable_2_3.launch.yaml max_solver_iterations:=6 use_mocap:=true
ros2 launch lab_7 deliverable_4.launch.yaml 2>&1 | tee deliverable_4.txt   # Ctrl+C після "final error"
ros2 run lab_7 deliverable_5 > deliverable_5.txt
```

// ═════════════════════════════════════════════════════════════════════
#set heading(numbering: "A.1")
#counter(heading).update(0)
= Словник позначень

#let gl(..rows) = table(
  columns: (auto, 1fr, auto), inset: 5pt, stroke: 0.4pt + luma(200),
  fill: (_, y) => if y == 0 { luma(235) },
  [*Символ*], [*Значення*], [*Розділ*],
  ..rows.pos().flatten(),
)

#gl(
  ([$X$, $X_i$], [усі змінні графа / змінні $i$-го фактора], [2.1]),
  ([$z_i$, $h_i$], [вимірювання та модель вимірювання], [2.1]),
  ([$Sigma_i$, $norm(dot)_Sigma$], [коваріація шуму, махаланобісова норма], [2.1]),
  ([$r(x)$, $J$], [вектор залишків, якобіан], [2.2]),
  ([$bold(delta)$, $lambda$, $D$], [крок оптимізатора, демпфування LM, масштабування], [2.2]),
  ([$W$, $L$], [вагова матриця, множник Холецького $W = L L^top$], [3]),
  ([$tilde(r)$, $tilde(J)$], [вибілені залишок і якобіан], [3]),
  ([$R$, $bold(t)$, $T$], [обертання, трансляція, поза $"SE"(3)$], [1.1]),
  ([$bold(omega)$, $bold(v)$, $bold(xi)$], [локальні координати: обертання, зсув, $[bold(omega); bold(v)]$], [2.3]),
  ([$⊕$, $"Exp"$, $"Log"$], [ретракція, експонента й логарифм групи], [2.3]),
  ([$H$], [якобіан фактора за локальними координатами (`evaluateError`)], [2.3]),
  ([$E(X)$], [`graph.error` — $1/2 sum norm(bold(e))^2_Sigma$], [2.5]),
  ([$T_i$, $R_i$, $bold(t)_i$], [вимірювання для усереднення], [4]),
  ([$M$, $U, S, V$], [сума обертань та її SVD (у 4.1); *(!)* у 2.3 немає], [4.1]),
  ([$cal(P)_("SO"(3))$], [проєкція на $"SO"(3)$], [4.1]),
  ([$omega_i$, $rho_i$], [ваги (концентрація Ланжевена, обернена дисперсія трансляції)], [4.2]),
  ([$Omega_i$], [вагова матриця $4 times 4$ зваженої хордальної норми], [4.2]),
  ([$c(omega)$], [нормувальна константа Ланжевена], [4.3]),
  ([$J_R$, $bold(r)_k$], [якобіан $op("vec")(R "Exp"(bold(omega)))$; стовпці $R$], [4.4, 4.5]),
  ([$A_i$, $bold(w)(A)$], [$R^top R_i$ і вектор його антисиметричної частини], [4.4]),
  ([$x_i$, $l_j$], [пози камер і точки в BA], [9]),
  ([$bold(u)_(i j)$, $pi$], [виміряний піксель і функція проєкції], [9]),
  ([$sigma_u$], [піксельний шум], [9]),
)

#v(1em)
#align(center, text(size: 9pt, fill: luma(120))[Джерела: завдання курсу VNAV (MIT, CC BY 4.0); F. Dellaert, _Factor Graphs and GTSAM: A Hands-on Introduction_; GTSAM 4.2 source; конспекти VNAV, лекції 15–16.])
