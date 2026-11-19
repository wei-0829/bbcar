"""
基於容量限制與十字網格地圖之循線自走車貨物分批配送路線規劃系統
A Capacity-Constrained Batch Delivery Route Planning System for Line-Following Autonomous Vehicles on Grid Maps

地圖：十字網狀地圖，倉庫起點 (0, -grid_range)（地圖最正下方），Manhattan 距離
我們的方法：容量感知節省值分批路徑改善法（ Capacity-Aware Savings-Based Batch Route Improvement ），簡稱 CAS-BRI
演算法：FIFO / Greedy / Sweep / CAS-BRI 四種比較
"""

import random
import math
import time
from itertools import combinations

# ═══════════════════════════════════════════════════════
# 資料結構
# ═══════════════════════════════════════════════════════

class Shelf:
    def __init__(self, sid, x, y):
        self.id = sid
        self.x  = x
        self.y  = y
    def __repr__(self):
        return f"S{self.id}({self.x},{self.y})"

class Cargo:
    def __init__(self, cid, shelf_id):
        self.id       = cid
        self.shelf_id = shelf_id
    def __repr__(self):
        return f"C{self.id}→S{self.shelf_id}"

# ═══════════════════════════════════════════════════════
# 距離 & 路徑工具（Manhattan）
# ═══════════════════════════════════════════════════════

def manhattan(ax, ay, bx, by):
    return abs(ax - bx) + abs(ay - by)

def route_distance(shelf_list, shelf_map, depot=(0, 0)):
    """depot 出發 → 依序走完貨架 → 回 depot 的總 Manhattan 距離"""
    if not shelf_list:
        return 0
    dx, dy = depot
    dist = manhattan(dx, dy, shelf_map[shelf_list[0]].x, shelf_map[shelf_list[0]].y)
    for i in range(len(shelf_list) - 1):
        a = shelf_map[shelf_list[i]]
        b = shelf_map[shelf_list[i + 1]]
        dist += manhattan(a.x, a.y, b.x, b.y)
    last = shelf_map[shelf_list[-1]]
    dist += manhattan(last.x, last.y, dx, dy)
    return dist

# ═══════════════════════════════════════════════════════
# 路徑改善：L<=5 窮舉全排列；L>5 使用 2-opt
# ═══════════════════════════════════════════════════════

from itertools import permutations as _permutations

def _brute_force(route, shelf_map, depot=(0, 0)):
    """窮舉全排列，保證找到最短路徑。時間複雜度 O(L!)"""
    best_route = route
    best_dist  = route_distance(route, shelf_map, depot)
    for perm in _permutations(route):
        d = route_distance(list(perm), shelf_map, depot)
        if d < best_dist - 1e-9:
            best_dist  = d
            best_route = list(perm)
    return best_route, best_dist

def _two_opt(route, shelf_map, depot=(0, 0), max_iter=1000):
    """2-opt 迭代改善，適用 L>5 的較長路徑。時間複雜度 O(L²) per iter"""
    best      = route[:]
    best_dist = route_distance(best, shelf_map, depot)
    improved  = True
    iters     = 0
    while improved and iters < max_iter:
        improved = False
        iters   += 1
        for i in range(len(best) - 1):
            for j in range(i + 2, len(best)):
                new_route = best[:i] + best[i:j+1][::-1] + best[j+1:]
                new_dist  = route_distance(new_route, shelf_map, depot)
                if new_dist < best_dist - 1e-9:
                    best      = new_route
                    best_dist = new_dist
                    improved  = True
    return best, best_dist

def optimize_route(route, shelf_map, depot=(0, 0)):
    """
    路徑改善統一入口：
      L <= 2 ：無需排列，直接回傳
      L <= 5 ：窮舉全排列（最多 5!=120 種），保證找到單一歸貨貨單內最優解
      L >  5 ：2-opt 迭代改善，避免 L! 爆炸
    """
    if len(route) <= 2:
        return route, route_distance(route, shelf_map, depot)
    if len(route) <= 5:
        return _brute_force(route, shelf_map, depot)
    return _two_opt(route, shelf_map, depot)

# ═══════════════════════════════════════════════════════
# 隨機產生問題
# ═══════════════════════════════════════════════════════

def generate_problem(m, n, q, grid_range=10, seed=42):
    random.seed(seed)
    depot = (0, -grid_range)   # 倉庫起點固定在地圖最正下方
    positions = set()
    while len(positions) < n:
        x = random.randint(-grid_range, grid_range)
        y = random.randint(-grid_range, grid_range)
        if (x, y) != depot:
            positions.add((x, y))
    shelves   = [Shelf(i, x, y) for i, (x, y) in enumerate(positions)]
    shelf_map = {s.id: s for s in shelves}
    cargos    = [Cargo(i, random.randint(0, n - 1)) for i in range(m)]
    return shelves, shelf_map, cargos, depot

# ═══════════════════════════════════════════════════════
# 演算法 1：FIFO（先到先處理）
# ═══════════════════════════════════════════════════════

def fifo_algorithm(cargos, shelf_map, q, depot=(0, 0)):
    """
    按照貨物編號順序（C0, C1, C2, ...）依序取 q 個分成一趟，
    直到所有貨物分完。每趟內貨架以首次出現順序訪問（去重）。
    不做任何路徑改善，作為最單純的基線對照。
    時間複雜度：O(m)，m=貨物數
    """
    trips = []
    for start in range(0, len(cargos), q):
        batch = cargos[start:start + q]
        # 依出現順序去重：同一趟若有多個貨物送同一貨架，只停一次
        route = list(dict.fromkeys(c.shelf_id for c in batch))
        trips.append(route)
    return trips

# ═══════════════════════════════════════════════════════
# 演算法 2：Greedy（最近鄰貪心）
# ═══════════════════════════════════════════════════════

def greedy_algorithm(cargos, shelf_map, q, depot=(0, 0)):
    """
    每次從當前位置出發，貪心找最近需配送的貨架，裝載直到滿 q 或無貨。
    時間複雜度：O(m × n)，m=貨物數，n=貨架數
    執行時間優化：remaining 用 set (O(1) discard)；預建 shelf_to_idxs 字典避免全掃。
    """
    shelf_to_idxs: dict = {}
    for i, c in enumerate(cargos):
        shelf_to_idxs.setdefault(c.shelf_id, set()).add(i)

    remaining: set = set(range(len(cargos)))
    trips = []

    while remaining:
        trip         = []
        cur_x, cur_y = depot

        while len(trip) < q and remaining:
            # 找最近需配送貨架
            best_shelf = None
            best_dist  = float('inf')
            for sid, idxs in shelf_to_idxs.items():
                if not idxs:
                    continue
                s = shelf_map[sid]
                d = manhattan(cur_x, cur_y, s.x, s.y)
                if d < best_dist:
                    best_dist  = d
                    best_shelf = sid
            if best_shelf is None:
                break

            # 批次裝載同貨架貨物
            idxs_for_shelf = shelf_to_idxs[best_shelf]
            for i in list(idxs_for_shelf):
                if len(trip) >= q:
                    break
                trip.append(best_shelf)
                idxs_for_shelf.discard(i)
                remaining.discard(i)

            s = shelf_map[best_shelf]
            cur_x, cur_y = s.x, s.y

        trips.append(list(dict.fromkeys(trip)))

    return trips

# ═══════════════════════════════════════════════════════
# 演算法 3：Sweep Algorithm（掃描法）
# ═══════════════════════════════════════════════════════

def sweep_algorithm(cargos, shelf_map, q, depot=(0, 0)):
    """
    Step 1: 將每個貨架轉換為極座標角度 atan2(y, x)
    Step 2: 按角度排序所有待送貨物
    Step 3: 依序將貨物分配到歸貨貨單，滿 q 則開新趟
    Step 4: 每趟路徑改善（L<=5 窮舉全排列；L>5 用 2-opt）
    時間複雜度：O(m logm + T×L! or T×L²)，m=貨物數，T=歸貨貨單數，L=歸貨貨單站數

    核心思想：角度相近的貨架在地理上相鄰，先掃完同一扇形區域
    再換下一區，可減少跨象限繞路。
    """
    # Step 1 & 2：按貨架角度排序所有貨物
    def shelf_angle(sid):
        s = shelf_map[sid]
        return math.atan2(s.y, s.x)   # [-π, π]

    sorted_cargos = sorted(range(len(cargos)),
                           key=lambda i: shelf_angle(cargos[i].shelf_id))

    # Step 3：依序分批（每批 q 個貨物）
    trips = []
    for start in range(0, len(sorted_cargos), q):
        batch = sorted_cargos[start:start + q]
        # 去重保留角度順序
        route = list(dict.fromkeys(cargos[i].shelf_id for i in batch))
        trips.append(route)

    # Step 4：每趟路徑改善（L<=5 窮舉，L>5 用 2-opt）
    optimized = []
    for trip in trips:
        if len(trip) > 2:
            opt_trip, _ = optimize_route(trip, shelf_map, depot)
        else:
            opt_trip = trip
        optimized.append(opt_trip)

    return optimized

# ═══════════════════════════════════════════════════════
# 演算法 4：CAS-BRI（我們的方法）
# ═══════════════════════════════════════════════════════

def saving_algorithm(cargos, shelf_map, q, depot=(0, 0)):
    """
    Step 1: 超量站點（單一貨架貨物數 >= q）直接成整趟歸貨貨單
    Step 2: Clarke-Wright Saving 合併剩餘站點
            saving(i,j) = d(depot,i) + d(depot,j) - d(i,j)
            由大到小合併，不超過載貨容量 q
    Step 3: 無法合併的貨物各自單獨成一趟
    Step 4: 路徑改善（L<=5 窮舉全排列；L>5 用 2-opt）
    時間複雜度：O(n²logn + T×L! or T×L²)，n=站點數，T=歸貨貨單數，L=歸貨貨單站數
    """
    # 統計每個貨架的貨物 index
    shelf_cargo_map = {}
    for i, c in enumerate(cargos):
        shelf_cargo_map.setdefault(c.shelf_id, []).append(i)

    trips      = []
    used_cargo = set()

    # Step 1：貨價需求的貨物數量超量處理
    for sid, idxs in shelf_cargo_map.items():
        while len(idxs) >= q:
            batch = idxs[:q]
            idxs  = idxs[q:]
            trips.append([sid])
            used_cargo.update(batch)
        shelf_cargo_map[sid] = idxs

    # Step 2：Clarke-Wright Saving
    active_shelves = [sid for sid, idxs in shelf_cargo_map.items() if idxs]

    def depot_dist(sid):
        s = shelf_map[sid]
        return manhattan(depot[0], depot[1], s.x, s.y)

    savings = []
    for i, j in combinations(active_shelves, 2):
        si, sj = shelf_map[i], shelf_map[j]
        sv = depot_dist(i) + depot_dist(j) - manhattan(si.x, si.y, sj.x, sj.y)
        savings.append((sv, i, j))
    savings.sort(reverse=True)

    route_of   = {}
    routes     = {}
    route_load = {}

    def new_route(sid):
        rid = len(routes)
        while rid in routes:   # 避免 id 碰撞
            rid += 1
        routes[rid]     = [sid]
        route_load[rid] = len(shelf_cargo_map.get(sid, []))
        route_of[sid]   = rid

    for sid in active_shelves:
        new_route(sid)

    for (sv, i, j) in savings:
        if i not in route_of or j not in route_of:
            continue
        ri, rj = route_of[i], route_of[j]
        if ri == rj:
            continue
        if route_load[ri] + route_load[rj] > q:
            continue
        for sid in routes[rj]:
            route_of[sid] = ri
        routes[ri].extend(routes[rj])
        route_load[ri] += route_load[rj]
        del routes[rj]
        del route_load[rj]

    for rid, shelf_list in routes.items():
        cargo_idxs = []
        for sid in shelf_list:
            cargo_idxs.extend(shelf_cargo_map.get(sid, []))
        if cargo_idxs:
            used_cargo.update(cargo_idxs)
            trips.append(list(dict.fromkeys(shelf_list)))

    # Step 3：剩餘無法合併的貨物各自單獨成一趟
    for i in range(len(cargos)):
        if i not in used_cargo:
            trips.append([cargos[i].shelf_id])

    # Step 4：路徑改善（L<=5 窮舉全排列；L>5 用 2-opt）
    optimized = []
    for trip in trips:
        if len(trip) > 2:
            opt_trip, _ = optimize_route(trip, shelf_map, depot)
        else:
            opt_trip = trip
        optimized.append(opt_trip)

    return optimized

# ═══════════════════════════════════════════════════════
# 評估
# ═══════════════════════════════════════════════════════

def evaluate(trips, shelf_map, q, cargos, depot=(0, 0)):
    total_dist    = sum(route_distance(t, shelf_map, depot) for t in trips)
    num_trips     = len(trips)
    total_cargo   = len(cargos)
    avg_load_rate = (total_cargo / (num_trips * q) * 100) if num_trips > 0 else 0
    return total_dist, num_trips, avg_load_rate


# ═══════════════════════════════════════════════════════
# 輸出排版工具：支援中文寬度的表格列印
# ═══════════════════════════════════════════════════════
import unicodedata

def _display_width(text):
    """計算字串在終端機中的顯示寬度，避免中文造成表格歪掉。"""
    text = str(text)
    width = 0
    for ch in text:
        if unicodedata.east_asian_width(ch) in ("W", "F"):
            width += 2
        else:
            width += 1
    return width

def _pad_right(text, width):
    text = str(text)
    return text + " " * max(0, width - _display_width(text))

def _pad_left(text, width):
    text = str(text)
    return " " * max(0, width - _display_width(text)) + text

def _print_table(headers, rows, aligns=None):
    """列印對齊表格。aligns 可放 'left' 或 'right'。"""
    if aligns is None:
        aligns = ["left"] * len(headers)

    all_rows = [headers] + rows
    widths = []
    for col in range(len(headers)):
        widths.append(max(_display_width(row[col]) for row in all_rows) + 2)

    def fmt_row(row):
        cells = []
        for i, cell in enumerate(row):
            if aligns[i] == "right":
                cells.append(_pad_left(cell, widths[i]))
            else:
                cells.append(_pad_right(cell, widths[i]))
        return "│ " + " │ ".join(cells) + " │"

    line = "├" + "┼".join("─" * (w + 2) for w in widths) + "┤"
    top = "┌" + "┬".join("─" * (w + 2) for w in widths) + "┐"
    bottom = "└" + "┴".join("─" * (w + 2) for w in widths) + "┘"

    print(top)
    print(fmt_row(headers))
    print(line)
    for row in rows:
        print(fmt_row(row))
    print(bottom)

# ═══════════════════════════════════════════════════════
# 執行並比較四種演算法
# ═══════════════════════════════════════════════════════

def run_comparison(m, n, q, grid_range=10, seed=42):
    shelves, shelf_map, cargos, depot = generate_problem(m, n, q, grid_range, seed)

    W = 72
    print("=" * W)
    print("  基於容量限制與十字網格地圖之循線自走車貨物分批配送路線規劃系統  —  四種演算法比較")
    print("=" * W)
    print(f"  貨物數 m = {m}　貨架數 n = {n}　載貨容量上限 q = {q}")
    print(f"  地圖範圍 [{-grid_range}, {grid_range}]　倉庫起點 {depot}　隨機種子 seed = {seed}")
    print("-" * W)

    print("\n【貨架位置】")
    for s in shelves:
        print(f"  {s}")
    print("\n【貨物分配（最多顯示 20 筆）】")
    for c in cargos[:20]:
        print(f"  {c}")
    if len(cargos) > 20:
        print(f"  ... 共 {len(cargos)} 筆")

    # ── 各演算法執行 ─────────────────────────────────
    algos = [
        ("演算法 1：FIFO（先到先處理）",        lambda: fifo_algorithm(cargos, shelf_map, q, depot)),
        ("演算法 2：Greedy（最近鄰貪心）",     lambda: greedy_algorithm(cargos, shelf_map, q, depot)),
        ("演算法 3：Sweep（掃描法）",           lambda: sweep_algorithm(cargos, shelf_map, q, depot)),
        ("演算法 4：CAS-BRI（我們的方法）",  lambda: saving_algorithm(cargos, shelf_map, q, depot)),
    ]

    complexity = [
        ("O(m)",              "m = 貨物數"),
        ("O(m × n)",          "m = 貨物數, n = 貨架數"),
        ("O(m × logm + t × L²)",  "m = 貨物數, t = 歸貨貨單數, L = 歸貨貨單站數"),
        ("O(n² × logn + t × L²)",  "n = 站點數, t = 歸貨貨單數, L = 歸貨貨單站數"),
    ]

    results = []
    for (label, fn), (cpx, cpx_note) in zip(algos, complexity):
        print(f"\n{'=' * W}")
        print(f"  {label}")
        print("=" * W)
        t0    = time.perf_counter()
        trips = fn()
        t1    = time.perf_counter()
        elapsed = t1 - t0
        dist, ntrips, rate = evaluate(trips, shelf_map, q, cargos, depot)

        for i, trip in enumerate(trips, 1):
            d = route_distance(trip, shelf_map, depot)
            print(f"    歸貨貨單 {i:3d}：{trip}  距離 = {d}")

        print(f"\n  總距離      : {dist}")
        print(f"  歸貨貨單數  : {ntrips}")
        print(f"  平均載貨率  : {rate:.1f}%")
        print(f"  時間複雜度  : {cpx}  ({cpx_note})")
        print(f"  執行時間    : {elapsed*1000:.3f} ms")

        results.append({
            "label":   label.split("：")[1],
            "dist":    dist,
            "trips":   ntrips,
            "rate":    rate,
            "time_ms": elapsed * 1000,
            "cpx":     cpx,
        })

    # ── 比較摘要表 ───────────────────────────────────
    print(f"\n{'=' * W}")
    print("  比較摘要")
    print("=" * W)

    min_dist = min(r["dist"] for r in results)
    max_rate = max(r["rate"] for r in results)
    min_time = min(r["time_ms"] for r in results)

    summary_headers = ["指標"] + [r["label"] for r in results]
    summary_rows = []

    summary_rows.append(["總距離"] + [
        f"{r['dist']}{' ★' if r['dist'] == min_dist else ''}" for r in results
    ])
    summary_rows.append(["歸貨貨單數"] + [str(r["trips"]) for r in results])
    summary_rows.append(["平均載貨率"] + [
        f"{r['rate']:.1f}%{' ★' if r['rate'] == max_rate else ''}" for r in results
    ])
    summary_rows.append(["執行時間（ms）"] + [
        f"{r['time_ms']:.3f}{' ★' if r['time_ms'] == min_time else ''}" for r in results
    ])
    summary_rows.append(["時間複雜度"] + [r["cpx"] for r in results])

    _print_table(
        summary_headers,
        summary_rows,
        aligns=["left"] + ["right"] * len(results)
    )
    print("  註：★ 代表該指標表現最佳。")

    # ── CAS-BRI 優勢分析 ──────────────────────────────
    saving_r = results[3]   # CAS-BRI 是主要方法
    print(f"\n  {'─' * (W - 2)}")
    print("  CAS-BRI 相對各演算法的改善（以總距離比較）")
    print(f"  {'─' * (W - 2)}")
    for r in results[:3]:
        if r["dist"] > 0:
            pct = (r["dist"] - saving_r["dist"]) / r["dist"] * 100
            if pct >= 0:
                print(f"    vs {r['label']}：改善 {pct:.1f}%  "
                      f"（{r['dist']} → {saving_r['dist']}，減少 {r['dist'] - saving_r['dist']}）")
            else:
                print(f"    vs {r['label']}：增加 {abs(pct):.1f}%  "
                      f"（{r['dist']} → {saving_r['dist']}，增加 {saving_r['dist'] - r['dist']}）")

    # ── 時間複雜度表格 ───────────────────────────────
    print(f"\n  【時間複雜度對比】")
    complexity_rows = []
    for r, (cpx, note) in zip(results, complexity):
        complexity_rows.append([r["label"], cpx, note])
    _print_table(
        ["演算法", "時間複雜度", "符號說明"],
        complexity_rows,
        aligns=["left", "left", "left"]
    )

    # 最終勝者
    best = min(results, key=lambda r: r["dist"])
    print(f"\n  ✅ 總距離最短演算法：{best['label']}")
    print("=" * W)

# ═══════════════════════════════════════════════════════
# 主程式（互動輸入）
# ═══════════════════════════════════════════════════════

if __name__ == "__main__":
    print("╔══════════════════════════════════════════════════════════════════════════════════╗")
    print("║  基於容量限制與十字網格地圖之循線自走車貨物分批配送路線規劃系統 (4 - Algorithm)  ║")
    print("╚══════════════════════════════════════════════════════════════════════════════════╝")
    print()

    try:
        m = int(input("請輸入貨物數量 m："))
        n = int(input("請輸入貨架數量 n："))
        q = int(input("請輸入自走車每趟最大載貨容量 q："))

        mode = input(
            "\n貨架位置輸入方式？\n"
            "  [1] 手動輸入\n"
            "  [2] 隨機產生（建議）\n"
            "選擇（1/2）："
        ).strip()

        if mode == "1":
            shelves   = []
            shelf_map = {}
            print(f"\n請輸入 {n} 個貨架的位置（格式：x y）：")
            for i in range(n):
                parts = input(f"  貨架 {i}：").split()
                x, y  = int(parts[0]), int(parts[1])
                s     = Shelf(i, x, y)
                shelves.append(s)
                shelf_map[i] = s

            depot  = (0, -max(abs(s.y) for s in shelves) if shelves else 0)  # 手動模式：depot 取所有貨架 y 最小值再往下一格
            cargos = []
            print(f"\n請輸入 {m} 個貨物的目標貨架（0 ~ {n-1}）：")
            for i in range(m):
                sid = int(input(f"  貨物 {i} 送往貨架編號："))
                cargos.append(Cargo(i, sid))

            algos = [
                ("FIFO",          lambda: fifo_algorithm(cargos, shelf_map, q, depot)),
                ("Greedy",        lambda: greedy_algorithm(cargos, shelf_map, q, depot)),
                ("Sweep",         lambda: sweep_algorithm(cargos, shelf_map, q, depot)),
                ("CAS-BRI", lambda: saving_algorithm(cargos, shelf_map, q, depot)),
            ]
            for name, fn in algos:
                print(f"\n{'='*50}\n  {name}\n{'='*50}")
                t0    = time.perf_counter()
                trips = fn()
                t1    = time.perf_counter()
                dist, nt, rate = evaluate(trips, shelf_map, q, cargos, depot)
                for i, trip in enumerate(trips, 1):
                    print(f"  貨單 {i}: {trip}  距離 = {route_distance(trip, shelf_map, depot)}")
                print(f"  總距離 = {dist}  歸貨貨單數 = {nt}  載貨率 = {rate:.1f}%  時間 = {(t1-t0)*1000:.3f}ms")

        else:
            seed_input = input("\n隨機種子（直接 Enter 使用預設 42）：").strip()
            seed       = int(seed_input) if seed_input else 42
            grid_input = input("地圖範圍（直接 Enter 使用預設 10，即 [-10, 10]）：").strip()
            grid_range = int(grid_input) if grid_input else 10
            run_comparison(m, n, q, grid_range=grid_range, seed=seed)

    except (ValueError, IndexError) as e:
        print(f"\n輸入錯誤：{e}")
        print("使用預設參數示範（m = 20, n = 8, q = 4）...")
        run_comparison(20, 8, 4)
