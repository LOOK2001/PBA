//-------------------------------------------------------
//
//  DynamicalState.h
//
//  Container for data associated with the dynamics
//  degrees of freedom in a system.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#ifndef ____PBA_DYNAMICALSTATE_H____
#define ____PBA_DYNAMICALSTATE_H____

#include "Vector.h"
#include "Color.h"
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <assert.h>


namespace pba
{

template<typename T>
class DSAttribute
{
  public:

    DSAttribute() : name("unknown") {}
    DSAttribute( const std::string& nam, const T& def ) : name(nam), defVal(def) {}
   ~DSAttribute(){}

    const size_t size() const { return data.size(); }
    const bool empty() const { return data.empty(); }
    void set(size_t i, const T& value ) { data[i] = value; }
    const T& get(size_t i ) const { return data[i]; }
    T& get(size_t i ) { return data[i]; }
    void expand_to( size_t n )
    {
       if( data.size() >= n ){ return; }
       size_t old_size = data.size();
       data.resize(n);
       for( size_t i=old_size;i<data.size();i++ )
       {
          data[i] = defVal;
       }
    }
    void clear() { data.clear(); }
    const std::string& attr_name() const { return name; }
    const T& default_value() const { return defVal; }
    typename std::vector<T>::const_iterator cbegin() const { return data.begin(); }
    typename std::vector<T>::const_iterator cend() const { return data.end(); }
    typename std::vector<T>::iterator begin() { return data.begin(); }
    typename std::vector<T>::iterator end() { return data.end(); }


  private:
    std::vector<T> data;
    std::string name;
    T defVal;
};


class DynamicalStateData 
{
  public:

    DynamicalStateData( const std::string& nam = "DynamicDataNoName");
    DynamicalStateData( const DynamicalStateData& d );
   ~DynamicalStateData();

    DynamicalStateData& operator= ( const DynamicalStateData& d );

    void create_attr( const std::string& nam, const int& def );
    void create_attr( const std::string& nam, const float& def );
    void create_attr( const std::string& nam, const Vector& def );
    void create_attr( const std::string& nam, const Color& def );

    const size_t add();
    const size_t add( const size_t nb );
    size_t nb() const;

    const int& get_int_attr( const std::string& nam, const size_t p ) const;
    const float& get_float_attr( const std::string& nam, const size_t p ) const;
    const Vector& get_vector_attr( const std::string& nam, const size_t p ) const;
    const Color& get_color_attr( const std::string& nam, const size_t p ) const;
    
    const Vector& pos( const size_t p ) const;
    const Vector& vel( const size_t p ) const;
    const Vector& accel( const size_t p ) const;
    const float& mass( const size_t p ) const;
    const int& id( const size_t p ) const;
    const Color& ci( const size_t p ) const;

    
    void set_attr( const std::string& nam, const size_t p, const int& value ); 
    void set_attr( const std::string& nam, const size_t p, const float& value ); 
    void set_attr( const std::string& nam, const size_t p, const Vector& value ); 
    void set_attr( const std::string& nam, const size_t p, const Color& value ); 

    void set_pos( const size_t p, const Vector& value );
    void set_vel( const size_t p, const Vector& value );
    void set_accel( const size_t p, const Vector& value );
    void set_mass( const size_t p, const float& value );
    void set_id( const size_t p, const int& value );
    void set_ci( const size_t p, const Color& value );

    std::vector<std::string> show_int_attrs() const;
    std::vector<std::string> show_float_attrs() const;
    std::vector<std::string> show_vector_attrs() const;
    std::vector<std::string> show_color_attrs() const;
    std::vector<std::string> show_all_attrs() const;

    bool attr_exists( const std::string& nam ) const;

    void merge( const DynamicalStateData& g );
	void clear();

    const std::string& Name() const { return name; }

    const double time() const { return t; }
    void update_time(const double dt){ t += dt; }

  protected:

    double t;
    size_t nb_items;

    // attribute collections
    std::map< std::string, DSAttribute<int>  > int_attributes;
    std::map< std::string, DSAttribute<float>  > float_attributes;
    std::map< std::string, DSAttribute<Vector> > vector_attributes;  
    std::map< std::string, DSAttribute<Color> > color_attributes;  

    std::map< std::string, DSAttribute<Vector> >::iterator    positions;
    std::map< std::string, DSAttribute<Vector> >::iterator    velocities;
    std::map< std::string, DSAttribute<Vector> >::iterator    accelerations;
    std::map< std::string, DSAttribute<float> >::iterator     masses;
    std::map< std::string, DSAttribute<int> >::iterator       ids;
    std::map< std::string, DSAttribute<Color> >::iterator     cis;

    std::string name;

    void re_find_main_attrs();


};

typedef std::shared_ptr<DynamicalStateData> DynamicalState;

DynamicalState CreateDynamicalState( const std::string& nam = "DynamicalDataNoName" );

DynamicalState copy( const DynamicalState d );

}
#endif
