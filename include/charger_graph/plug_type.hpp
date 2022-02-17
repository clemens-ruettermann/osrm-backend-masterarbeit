//
// Created by kenspeckle on 4/8/22.
//

#ifndef OSRM_PLUG_TYPE_HPP
#define OSRM_PLUG_TYPE_HPP

#include <unordered_map>
#include <string>

enum PlugType {
	AC_CEE_3_polig,
	AC_CEE_5_polig,
	AC_Kupplung_Typ_2,
	AC_Schuko,
	AC_Steckdose_Typ_2,
	DC_CHAdeMO,
	DC_Kupplung_Combo,
	DC_Kupplung_Tesla_Typ_2,
	Steckdose_Typ_1,
	Tesla,
	Typ_2
};

static std::unordered_map<std::string, PlugType> PLUG_TYPE_MAP {
		{"AC CEE 3 polig", PlugType::AC_CEE_3_polig},
		{"AC CEE 5 polig", PlugType::AC_CEE_5_polig},
		{"AC Kupplung Typ 2", PlugType::AC_Kupplung_Typ_2},
		{"AC Schuko", PlugType::AC_Schuko},
		{"AC Steckdose Typ 2", PlugType::AC_Steckdose_Typ_2},
		{"DC CHAdeMO", PlugType::DC_CHAdeMO},
		{"DC Kupplung Combo", PlugType::DC_Kupplung_Combo},
		{"DC Kupplung Tesla Typ 2", PlugType::DC_Kupplung_Tesla_Typ_2},
		{"Steckdose Typ 1", PlugType::Steckdose_Typ_1},
		{"Tesla", PlugType::Tesla},
		{"Typ 2", PlugType::Typ_2}
};


#endif //OSRM_PLUG_TYPE_HPP
