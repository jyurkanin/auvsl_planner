
typedef struct {
    float kc,kphi,n0,n1,phi;
    char *name;
} BekkerData;

BekkerData lookupSoilTable(int index);
BekkerData get_soil_data_at(float x, float y);
float get_altitude(float x, float y);
