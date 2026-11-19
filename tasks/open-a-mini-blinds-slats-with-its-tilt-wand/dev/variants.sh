#!/bin/bash
# Dev only: run the named BLIND_DEMO variants one after another and print each one's score and last reading.
# dev/variants.sh nothing one half ...   Created 11_17_26.
set -u
cd "$(dirname "$0")/.."
for v in "$@"; do
  steps=3000
  case "$v" in nothing) steps=1200;; one|yank|push) steps=700;; half) steps=1700;; far_over) steps=5400;; over) steps=3600;; esac
  BLIND_DEBUG=1 BLIND_DEMO=$v bash dev/run.sh "v-$v" "$steps" | tail -1
  grep -E "^BLIND t" ~/yam-eval/blind-v-$v/stdout.log | tail -1 | cut -c1-175
  grep -E "^BLIND t" ~/yam-eval/blind-v-$v/stdout.log | awk '{for(i=1;i<=NF;i++){if($i=="shift"&&$(i+1)>m)m=$(i+1); if($i=="plumb"&&$(i+1)>p)p=$(i+1)}} END{print "   greatest shift mm", m, "greatest wand swing deg", p}'
done
