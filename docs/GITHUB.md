# GitHub-Veröffentlichung

Zielname vom Benutzer bestätigt: SpaceHexer/SCARA-Robot-Controller-Serial.
Sichtbarkeit zunächst privat. Erstellungsversuch:

    gh repo create SpaceHexer/SCARA-Robot-Controller-Serial --private ...

Ergebnis dieser Umgebung: `Get https://api.github.com/users/SpaceHexer: Forbidden`.
Auch `gh api user` liefert Forbidden. Native Git-Lesezugriffe auf das alte
Repository funktionieren; daraus folgt keine GitHub-API-/Erstellungsberechtigung.
Keine Geheimnisse wurden ausgelesen oder in Dateien gespeichert. Es wurde keine
neue GitHub-URL als erfolgreich angelegt ausgegeben und kein Push durchgeführt.

Wenn GitHub hier wieder nutzbar ist, den bereits versionierten lokalen Stand
mit normalem Push veröffentlichen; niemals ein existierendes fremdes Repository
mit force-push überschreiben. Optional auf einem angemeldeten Computer nach
Entpacken der ZIP (die ZIP enthält absichtlich kein .git):

```bash
git init -b main
git add .
git commit -m "SCARA Serial controller with quintic joint profiles"
gh auth login
gh repo create SpaceHexer/SCARA-Robot-Controller-Serial --private --source . --remote origin --push
```

Die Befehle sind eine Anleitung, kein Nachweis einer hier erfolgten Veröffentlichung.
