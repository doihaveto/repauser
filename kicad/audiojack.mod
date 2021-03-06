PCBNEW-LibModule-V1  20/10/2013 19:46:17 PM
# encoding utf-8
Units mm
$INDEX
JACK_ALIM
$EndINDEX
$MODULE JACK_ALIM
Po 0 0 0 15 526408C9 00000000 ~~
Li JACK_ALIM
Cd module 1 pin (ou trou mecanique de percage)
Kw CONN JACK
Sc 0
AR /5262BE7F
Op 0 0 0
T0 0.254 -5.588 1.016 1.016 0 0.254 N V 21 N "R8"
T1 0.9 3.1 1.016 1.016 0 0.254 N V 21 N "FSR"
DS 7.6 -4.33 6.79 -4.33 0.15 21
DS 6.7 4.31 7.6 4.31 0.15 21
DS -1.34 0 -1.34 -4.318 0.15 21
DS -1.34 -4.328 6.788 -4.328 0.15 21
DS 7.608 -4.338 7.608 4.298 0.15 21
DS 6.788 4.308 -1.34 4.308 0.15 21
DS -1.33 4.308 -1.33 -0.01 0.15 21
$PAD
Sh "1" O 1.3 2.3 0 0 0
Dr 0.6 0 0
At STD N 00E0FFFF
Ne 1 "N-0000015"
Po 0 0
$EndPAD
$PAD
Sh "2" C 1.7 1.7 0 0 0
Dr 0.9 0 0
At STD N 00E0FFFF
Ne 2 "VCC"
Po 8.6 -3.6
$EndPAD
$PAD
Sh "3" O 1.3 2.3 0 0 0
Dr 0.6 0 0
At STD N 00E0FFFF
Ne 0 ""
Po 8.6 0
$EndPAD
$SHAPE3D
Na "connectors/POWER_21.wrl"
Sc 0.8 0.8 0.8
Of 0 0 0
Ro 0 0 0
$EndSHAPE3D
$EndMODULE JACK_ALIM
$EndLIBRARY
