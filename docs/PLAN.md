# Umgesetzter Plan – aktuelle Benutzeranforderungen

Die letzte ausführliche Benutzeranweisung gibt die Neufassung frei und ersetzt
sämtliche älteren Vorgaben zu Geraden oder kürzesten TCP-Bahnen.

1. Ziel-IK statt kartesischer Streckenplanung: implementiert. MOVEL/CHECKLINE
   ausdrücklich entfernt; X/Y per MOVEXY, anschließend Gelenkprofile.
2. Quintik je Gelenk mit allen sechs Randwerten: implementiert. BEGIN/SEG/RUN
   erlaubt ausführbare kontinuierliche Folgen. Endraten benötigen Nachfolge.
3. Gemeinsame Zeitplanung, innere Extremstellen und Achsgrenzen: implementiert.
   Allgemeine Randraten werden für neue Dauern refittet, begrenzte Suche.
4. Direkte STEP/DIR-Ausgabe, atomare Fehler-/Fertigmeldung: implementiert.
5. Homing je Achse, parallel oder sequenziell, zwei Antastungen: implementiert.
   Ergänzung 28.10.2026: SETREF ALL / SETREF J1/J2/J3/Z erlaubt bewusstes
   Überspringen der Schalterfahrt bei bekannter tatsächlicher Stellung. Keine
   Motorbewegung, keine automatische Boot-Referenz, STOP-Sperre bleibt wirksam.
   Physische Konfiguration/Freigabe am Roboter steht aus.
6. Zwei im Code schaltbare STOP-Taster auf freien Pins 22/23: implementiert.
   Disabled bedeutet kein GPIO-Zugriff, auch nicht bei Reset und Status.
7. Positionsspeicher: Benutzer hat RAM gewählt. 16 Slots, nach Neustart leer.
8. Bedienung, Float32-Hosttests und vier Mega-Build-Konfigurationen: siehe
   VALIDIERUNG.md. Keine physische Bewegung oder Firmwareprogrammierung ausgeführt.
9. GitHub: aktuelles Benutzerziel SpaceHexer/SCARACodeFinal. Schreibzugriff am
   28.10.2026 verfügbar; vorhandene Benutzerkonfiguration mit lokalen Änderungen
   zusammenführen, ohne Remote-Historie zu überschreiben.

Der alte Plan liegt ausschließlich als historischer Beleg in docs/archive.
