# Quintische Gelenkprofile und Zielkinematik

Keine kartesische Gerade und kein Bahnfortschritt s werden geplant.
Kinematics löst nur am Ziel x/y/z/psi die IK. Movement verwendet anschließend
je Achse ein Polynom der Gelenkkoordinate, mit gemeinsamer Segmentdauer.

## Sechs Randwerte je Gelenk

    u(t_lauf) = t_lauf / T_fahrt
    q_i(t_lauf) = Summe b_k,i * u(t_lauf)^k, k=0..5
    b0 = q_start
    b1 = T_fahrt*v_start
    b2 = T_fahrt²*a_start/2
    R = q_ziel-b0-b1-b2
    V = T_fahrt*v_ziel-b1-2*b2
    A = T_fahrt²*a_ziel-2*b2
    b3 = 10*R-4*V+A/2
    b4 = -15*R+7*V-A
    b5 = 6*R-3*V+A/2

R/V/A sind Residuen, keine Achsgrenzen. Jede Achse hat eigene b-Koeffizienten.
Geschwindigkeit, Beschleunigung und Ruck folgen durch Ableiten und Division
mit T_fahrt, T_fahrt² bzw. T_fahrt³. Die Endwerte werden zusätzlich direkt
gehalten, um vermeidbare Rundungsauslöschung am Endpunkt zu vermeiden.

Für Ruhe-zu-Ruhe gilt q=q_start+(q_ziel-q_start)*(10u³-15u⁴+6u⁵).
Die Maxima sind 1,875*|Delta_q|/T, (10*sqrt(3)/3)*|Delta_q|/T² und
60*|Delta_q|/T³. Daraus wird eine erste T_probe gewählt. Anschließend werden
immer die wirklichen Profile geprüft. Bei allgemeinen Randraten muss nach jeder
Zeitänderung neu gefittet werden; längere Zeit kann Überschwingen verstärken.

## Innere Extremstellen

Positionsmaxima liegen an Nullstellen des Geschwindigkeits-Polynoms (Grad 4),
Geschwindigkeitsmaxima an Beschleunigungsnullstellen (Grad 3), Beschleunigungs-
maxima an Rucknullstellen (Grad 2), Ruckmaxima an Snapnullstellen (Grad 1).
Die Root-Suche zerlegt [0,1] rekursiv an Ableitungsnullstellen in monotone
Intervalle und bisektiert Vorzeichenwechsel. Endpunkte werden stets geprüft.
Das ist eine numerische Float32-Extremstellenprüfung, keine formale gerichtete
Intervallarithmetik. Kleine Rundungstoleranzen sind explizit; Zielhardware bleibt
auf schwierige Grenzfälle zu prüfen. Dichte Stichproben dienen nur zur Testkontrolle.

Die letzte Folge muss mit v=a=0 enden. Anschlusssegmente übernehmen zuvor geprüfte
Referenzraten; explizite V0/A0 müssen dazu passen. Die erste Bewegung startet aus
Halten, nicht aus einer willkürlich behaupteten nicht-null Geschwindigkeit.

## Ziel-IK

    x_W = x_TCP - delta_x*cos(psi) + delta_y*sin(psi)
    y_W = y_TCP - delta_x*sin(psi) - delta_y*cos(psi)
    c = (x_W²+y_W²-L1²-L2²)/(2*L1*L2)
    theta2 = +/- acos(c)
    theta1 = atan2(y_W,x_W)-atan2(L2*sin(theta2),L1+L2*cos(theta2))
    theta3 = psi-theta1-theta2

Beide Zweige werden gegen konfigurierte und referenzbasierte Softgrenzen geprüft.
Gewählt wird eine zulässige, winkelmäßig nahe Lösung. Z ist für beide gleich und
wird nicht in einen gemischten mm/rad-Abstandsvergleich eingerechnet. theta1 wird
zur nächsten 2*pi-Darstellung entrollt. Ist die nicht zulässig, wird nicht heimlich
eine volle Umdrehung als Ersatz erzeugt. Große Gelenkfahrten ausdrücklich über
MOVEJ wählen. Die gleichlange-Arm/Ursprung-Degeneration wird gesondert erkannt.
Ein gestrecktes Ziel ist für Position-IK zulässig, da keine inverse Geschwindig-
keitskinematik oder kartesische Startkurve benötigt wird.

Die Werkzeugbahn zwischen diesen Zielgelenken wird nicht als Gerade geprüft.
Auch zulässige Zielgelenke und gültige Gelenkprofile garantieren keine Kollisionsfreiheit.
