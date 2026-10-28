# Validierung der neuen Gelenkraum-Firmware

Datum: 28.10.2026. Prüfung bezieht sich auf den neuen Code in diesem Repository,
nicht auf die frühere kartesische Serial-Firmware.

## Ausgeführt

- g++ C++11, -Wall -Wextra -Werror, AddressSanitizer und UndefinedBehaviorSanitizer.
- Float32-Rechenkern auch im Hosttest, entsprechend der AVR-Präzision.
- Je STOP-Konfiguration 00, 01, 10, 11 ein Subsystem-Testprogramm und ein
  Integrationsprogramm mit echter main-Schleife: insgesamt acht Testprogramme.
- PlatformIO/AVR-GCC Mega-Builds für megaatmega2560, mega_stop00, mega_stop01,
  mega_stop10. Keine Stepper- oder andere externe C++-Bibliothek.
- Standardbuild: 3181/8192 Bytes statischer RAM, 36812/253952 Bytes Flash.
- Alle vier AVR-Konfigurationen erfolgreich. Die aktivierte Arduino-Core-
  Warnstufe zeigt unused-parameter-Warnungen in framework new.cpp; eigene
  Projektdateien wurden ohne Compilerwarnungen gebaut.

## Abgedeckte Softwareeigenschaften

1. Alle sechs Randwerte und bekannte skalare Quintik-Fixtures; 100 allgemeine
   Randwertsätze. Position/Speed/Acceleration/Jerk-Extrema mit inneren Verletzungen.
   Zusätzlich 200 allgemeine Profile erzeugt; 157 akzeptierte Profile an je
   4001 Zeitpunkten unabhängig gegen Positions-/Geschwindigkeits-/Beschleunigungs-/
   Ruckgrenzen gegengeprüft.
2. Beide Ziel-IK-Zweige innerhalb zulässiger Grenzen, FK-Rückrechnung, Endpunkte
   im gestreckten Arm, Z-/Gelenkgrenzen und unerreichbare Punkte.
3. Impulsumrechnung, tatsächliche simulierte GPIO-Flanken, gemeinsames Frame-
   Timing, Pufferunterlauf und Abschaltung der Treiber.
4. Gemeinsame Segmentdauer, EXACT-Ablehnung, tatsächlich ausgeführte Folgen mit
   nicht-null End-/Startgeschwindigkeit UND -beschleunigung, Kontinuität und
   Nachfolgepflicht, auch für sehr kleine nicht-null Endraten.
5. Teilreferenzen: SETJ auf referenzierter Achse erlaubt, X/Y bei fehlenden
   Referenzen abgelehnt. Einzel-, paralleles und sequenzielles zweistufiges
   Homing mit simulierten mechanischen Schaltern, Freifahren, Nullpunkten, Timeout.
6. RAM-Speichergrenzen, leere Slots, Speichern/Laden/Löschen und leerer Neustart.
7. Boot mit aktiven STOPs, Verriegelung bis Freigabe, keine automatische
   Wiederaufnahme. Deaktivierte Pins: exakt null pinMode-/Read-/Write-Aufrufe.
8. Vollständiger main-/Serial-Ablauf: HOME -> MOVEXY ohne vorheriges MOVEJ -> SAVE
   -> Bewegung -> POSITION, sechs Randwerte über BEGIN/SEG/RUN, vollständiges LIST
   mit 16 belegten Slots, Ablehnung entfernter Befehle und ungültiger Zeilen.
9. Serial-STOP in allen vier Tasterkonfigurationen; aktivierte Taster während
   Homing, Planung und Fahrt, gemeinsame Betätigung, keine falsche DONE-Meldung.
10. Endschalterfehler während Fahrt, erneute Referenzpflicht und blockiertes
    RESETSTOP bei gedrücktem verbleibendem aktivem Taster.
11. SETREF für bekannte manuelle Stellung: Teilreferenzen, Grad/mm, Grenz- und
    Zahlenprüfung, keine STEP-Flanken oder Änderung der Treiberfreigabe,
    Ablehnung während vorbereiteter/laufender Bewegung sowie bei STOP/Timingfehler
    und aktivem STOP-Taster. ALL wirkt bei ungültiger Eingabe nicht teilweise.
12. Regressionsfall aus Benutzerprotokoll: abgelehntes HOME nach Timingfehler
    erzeugt kein erneutes DONE HOME. Der Test schlug vor der Korrektur fehl.
    Extreme Frameziele INT32_MIN/INT32_MAX werden ohne Integerüberlauf abgewiesen.

## AVR-Ausführung im Simulator, 28.10.2026

Der ursprüngliche Mega-Build aus Commit 2313dd9 wurde mit simavr 1.6 bei
16 MHz ausgeführt. Nach Setzen der Referenzbits ausschließlich im simulierten
RAM reproduzierte `MOVEJ 1 1 1 0 2` das gemeldete STOPPED TIMING_ERROR.
Gemessene maximale Timer3-ISR in dieser Simulation: 2580 CPU-Takte = 161,25 µs
bei 100 µs vorgesehenem Interruptabstand. Das ist ein Nachweis am simulierten
AVR-Code, keine Messung am physischen Mega.

Die neue Regression `tests/avr-timing/runner.c` führt die echte erzeugte ELF aus
und benutzt ausschließlich die Serial-Schnittstelle, einschließlich SETREF.
Keine internen Referenzbits werden im neuen Test überschrieben. Sie prüft den
ursprünglich fehlschlagenden Befehl, größere Vierachsfahrten, Richtungswechsel,
eine vorbereitete Folge mit nicht-null Randgeschwindigkeit/-beschleunigung,
gleichzeitige PING/STATUS-Kommunikation sowie STOP, blockiertes SETREF und
RESETSTOP. Fünf Bewegungsabschlüsse müssen erfolgreich sein; die gemessene
Timer3-ISR muss unter 1600 CPU-Takten (=100 µs) bleiben.

Alle vier Mega-Builds durchliefen dieses Szenario erfolgreich:

| STOP1 / STOP2 | Maximale Timer3-ISR im geprüften Szenario |
|---|---|
| 1 / 1 | 1401 CPU-Takte = 87,5625 µs |
| 0 / 0 | 1352 CPU-Takte = 84,5 µs |
| 0 / 1 | 1384 CPU-Takte = 86,5 µs |
| 1 / 0 | 1384 CPU-Takte = 86,5 µs |

Die Szenarien begrenzen nicht sämtliche denkbaren Laufzeitpfade. Insbesondere
ist dies keine Messung der Homing-ISR oder elektrischer Signale am Roboter.
Protokolle: `tests/results/avr-timing-*.log`.

Reproduktion unter Linux mit `libsimavr-dev`:

```bash
pio run -e megaatmega2560
bash tests/avr-timing/run.sh .pio/build/megaatmega2560/firmware.elf
```

Frame-Differenzen und Pulsgrenzen werden jetzt außerhalb der ISR vorbereitet.
Der beim Framewechsel garantiert impulsfreie erste DDA-Takt wird beim Laden
vorweggenommen; dadurch bleibt die Framezeit unverändert. Die Zeitüberwachung
bleibt aktiv. Hostsimulation und AVR-Simulation ergänzen sich; die bisherige
Hostsimulation allein konnte die reale AVR-Instruktionslaufzeit nicht prüfen.

Die Protokolle liegen unter tests/results/; nachgestellte Leerzeichen wurden entfernt. Die Hostsimulation ersetzt weder
Interruptpräemption auf echter Hardware noch eine elektrische Messung. Die
Numerikprüfung ist keine formale gerichtete Intervallarithmetik.

## Nicht ausgeführt / am Roboter offen

Kein Firmware-Upload und keine reale Bewegung. Kein Last-/Schrittverlusttest,
keine Encoder-Istmessung, keine Kollisionsprüfung, keine physische Messung von ISR-Laufzeit,
STEP-Pulsbreite und DIR-Vorlauf. HOME_CONFIRMED und PARALLEL_CONFIRMED bleiben
für echte Hardware standardmäßig deaktiviert, bis die Konfiguration geprüft ist.
Der Host-Testschalter SCARA_TEST_CONFIRMED darf nicht in AVR-Binaries verwendet
werden; ein Compilerfehler verhindert dies.

GitHub: Erstellung von SpaceHexer/SCARA-Robot-Controller-Serial versucht, durch
API-Forbidden blockiert. Kein Remote erstellt oder Upload behauptet. Lokaler
Git-Stand und ZIP enthalten den beauftragten Fallback.
