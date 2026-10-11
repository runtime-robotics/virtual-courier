#!/usr/bin/env python3
"""Сводная таблица по results/*.json.

По каждому сценарию у команды берётся лучшая попытка, итог — сумма по сценариям.
Баллы за защиту (до +20 %) добавляются из CSV: team,defense_percent

    tools/leaderboard.py [--results results] [--defense defense.csv] [--scenarios final_1,final_2]
"""

import argparse
import csv
import glob
import json
import os
from collections import defaultdict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', default=os.path.join(os.path.dirname(__file__), '..', 'results'))
    ap.add_argument('--defense', help='CSV: team,defense_percent (0–20)')
    ap.add_argument('--scenarios', help='учитывать только эти сценарии (через запятую)')
    args = ap.parse_args()

    only = set(args.scenarios.split(',')) if args.scenarios else None
    best = defaultdict(dict)   # team → scenario → result
    for path in glob.glob(os.path.join(args.results, '*.json')):
        with open(path, encoding='utf-8') as f:
            r = json.load(f)
        if only and r['scenario'] not in only:
            continue
        cur = best[r['team']].get(r['scenario'])
        if cur is None or (r['score'], -r['elapsed']) > (cur['score'], -cur['elapsed']):
            best[r['team']][r['scenario']] = r

    defense = {}
    if args.defense:
        with open(args.defense, encoding='utf-8') as f:
            for row in csv.reader(f):
                if row and row[0] != 'team':
                    defense[row[0]] = min(20.0, max(0.0, float(row[1])))

    scenarios = sorted({s for t in best.values() for s in t})
    rows = []
    for team, res in best.items():
        base = sum(r['score'] for r in res.values())
        bonus = round(base * defense.get(team, 0.0) / 100, 1)
        time_sum = sum(r['elapsed'] for r in res.values())
        rows.append((base + bonus, base, bonus, -time_sum, team, res))
    rows.sort(reverse=True)

    head = f"{'#':>2}  {'команда':<20}" + ''.join(f'{s:>12}' for s in scenarios) + f"{'сумма':>8}{'защита':>8}{'ИТОГ':>8}"
    print(head)
    print('-' * len(head))
    for place, (total, base, bonus, _, team, res) in enumerate(rows, 1):
        cells = ''.join(f"{res[s]['score'] if s in res else '—':>12}" for s in scenarios)
        print(f'{place:>2}  {team:<20}{cells}{base:>8}{bonus:>8}{total:>8}')


if __name__ == '__main__':
    main()
