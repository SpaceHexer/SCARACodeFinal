# Konfiguration am Roboter

Vom Benutzer beantwortet:
- Aktuelles Repository: SpaceHexer/SCARACodeFinal (Benutzerangabe 28.10.2026).
- Benutzerstand: HOME_CONFIRMED für alle Achsen true, paralleles Homing false;
  sequenzielle Reihenfolge Z → J1 → J2 → J3. Beim Zusammenführen erhalten.
- Positionen ausschließlich RAM, kein EEPROM.
- L1=228 mm, L2=136,5 mm; Werkzeugversatz aktuell 0/0.
- Geometrische Referenzwinkel vorläufig 0° bei allen Drehachsen.
- Einzel-, vollständig paralleles und sequenzielles Homing gewünscht.

Vom Benutzer anschließend am Roboter einzustellen/zu prüfen:
1. qRef und homeOffsetSteps zusammen mit echter Endschalterlage, homeDirection,
   direction, Microsteps/Getriebe und Z-Skalierung. Die Softgrenze am Referenz-
   Endschalter wird daraus zusätzlich zu minimum/maximum abgeleitet.
2. Seek-/Latchgeschwindigkeit, Beschleunigung, Freifahrweg und Zeit-/Weggrenzen.
3. HOME_CONFIRMED einzeln setzen; PARALLEL_CONFIRMED und PARALLEL_GROUPS nur nach
   Prüfung mechanisch zulässiger Gleichzeitigkeit freigeben.
4. STOP-Schalter Pins 22/23 gegen GND, aktiv LOW/PULLUP; bei Nichtverwendung die
   betreffende constexpr-Option false setzen. Alle aktiven Pins werden geprüft.
5. Treibertyp, STEP/DIR-Minimalzeiten und Z-Halten/Bremse bei Treiberabschaltung.
6. Reale Lastgrenzen, Genauigkeit und ISR-Laufzeit/Pulszeiten messen.

Die Softwaretests bestätigen keine kollisionsfreie Bewegung, gemessenen Istwinkel
oder Last-/Haltefähigkeit. Es gibt keine pauschale kartesische Start-Singularitäts-
sperre mehr; die eigentlichen Zielgelenkgrenzen bleiben bindend.
