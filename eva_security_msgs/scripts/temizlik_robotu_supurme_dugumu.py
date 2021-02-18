#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from eva_security_msgs.msg import DistanceInfo
from geometry_msgs.msg import Twist
from eva_security_msgs.srv import *


#__________________________________________________________________________________________________
#
#  TemizlikRobotuSupurme
#
#  AMAÇ:    Robotun hareket eylemini gerçekleştirmek için
#           oluşturulmuş sınıftır.
#
#  SINIF DEĞİŞKENLERİ:
#
# 	### Constructor değişkenleridir. İlk ve varsayılan değerleri
#   ### aktarılmaktadır.
#
#   self.mesafe_bilgisi_hesaplanan_mesafe
#   self.robot_hareket_emir
#   self.toplam_gidilmesi_gereken_mesafe
#   self.tamamlanan_gorevlerin_toplam_mesafesi
#   self.gorev_listesi
#   self.mevcut_gorev
#   self.robot_donus_yap_emri
#   self.tamamlanan_gorev_yuzde_degeri
#   self.robot_bas_aci
#   self.robot_bas_aci_tolerans_degeri
#   self.acisal_referans_hizi
#
#
#
#  FONKSİYON PROTOTİPLERİ:
#
# 	// Constructor
# 	def __init__(self):
#
# 	def ana_fonksiyon(self):
#   def gorev_okuma_fonksiyonu(self):
#   def gorev_atama_fonksiyonu(self):
#   def robot_durma_fonksiyonu(self, robot_hiz_mesaji):
#   def robot_gorev_uygulama_fonksiyonu(self, robot_hiz_mesaji):
#   def robot_donus_yapma_fonksiyonu(self, hedef_bas_aci_derece):
#   def bas_aci_callback_fonksiyonu(self, odom_mesaji):
#   def engel_bilgilendirme_servisi(self, istek):
#   def gorev_bilgilendirme_servisi(self, istek):
#   def mesafe_hesabi_callback_fonksiyonu(self, odom_mesaji):
#
#
#
#   NOTLAR:
#
#   GELİŞTİRME GEÇMİŞİ:
#
#   Yazar:
#   Tarih: 10.02.2020
#   Versiyon: v_1.0
#
#
#__________________________________________________________________________________________________

class TemizlikRobotuSupurme(object):

    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              __init__
    #   FONKSIYON AÇIKLAMASI:       Constructor fonksiyodur. Bütün değişkenlere
    #    							ilk ve varsayılan değerleri atanmaktadır.
    #
    #
    #   DEĞİŞKENLER:
    #       ADI                                     TIPI            AÇIKLAMASI
    #   mesafe_bilgisi_hesaplanan_mesafe            int             mesafe_hesabi_callback_fonksiyonundan dinlendiği gidilen toplam mesafe bilgisi değerini ifade eder.
    #   robot_hareket_emir                          bool            engel_bilgilendirme_servisinden gelen istek değerini ifade eder. Bu değer False ise engelin var olduğunu belirtir.
    #   toplam_gidilmesi_gereken_mesafe             int             Görev listesinden belirtilen görevlerdeki mesafe değerlerinin toplamını ifade eder.
    #   tamamlanan_gorevlerin_toplam_mesafesi       int             Tamamlanmış görevlerin toplam mesafe değerlerini ifade eder.
    #   gorev_listesi                               list            Görevler yaml dosyasından okunmuş görevleri içermektedir.
    #   mevcut_gorev                                int             Mevcut görevi belli eder. Görevler tamamlandıkça değeri artmaktadır.
    #   robot_donus_yap_emri                        bool            Mevcut görev tamamlandıktan sonra uygun baş açıya dönüş yapması ifade eder. 
    #                                                               True ise görevin bittiğini ve robot_donus_yapma_fonksiyonu çağırması gerektiğini ifade eder.
    #   tamamlanan_gorev_yuzde_degeri               float           toplam_gidilmesi_gereken_mesafe değeri ve mesafe_bilgisi_hesaplanan_mesafe değerini oranlayarak görevlerin 
    #                                                               % olarak ne kadar tamamlanmış olduğunu ifade eder.
    #   robot_bas_aci                               float           bas_aci_callback_fonksiyonu fonksiyonundan dinlediği baş açı değerini ifade ederi.
    #   robot_bas_aci_tolerans_degeri               float           Parametreler yaml dosyasından okunmuş Bas Aci Referans Degeri değerini ifade eder.
    #                                                               Robot için hedeflenen baş açı değeri ile robotun mevcut baş açı değeri arasındaki farkın bu değerden az olması gerekmektedir.
    #                                                               Bu tolerans değerine kadar robot baş açısı için açısal hız üretmektedir.
    #   acisal_referans_hizi                        float           Parametreler yaml dosyasından okunmuş Acisal Referans Hiz Degeri değerini ifade eder.
    #                                                               Robotun açısal hız üretebilmesi için gereken referans hız değeridir.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def __init__(self):
        self.mesafe_bilgisi_hesaplanan_mesafe = 0
        self.robot_hareket_emir = True

        self.toplam_gidilmesi_gereken_mesafe = 0
        self.tamamlanan_gorevlerin_toplam_mesafesi = 0
        self.gorev_listesi = list()
        self.gorev_atama_fonksiyonu()
        self.mevcut_gorev = 0

        self.robot_donus_yap_emri = False

        self.tamamlanan_gorev_yuzde_degeri = 0

        self.robot_bas_aci = 0
        self.robot_bas_aci_tolerans_degeri = rospy.get_param("~Parameters/head_angle_referance")
        self.acisal_referans_hizi = rospy.get_param("~Parameters/angular_speed_referance")

        self.ana_fonksiyon()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              ana_fonksiyon
    #   FONKSIYON AÇIKLAMASI:       Yayıncılar ve istemciler dinlenerek okunan değerlere göre "/cmd_vel"
    #                               topiğinden hareket hızı değerlerini yayınlanmaktadır.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "/odom", "/mesafe_hesabi" topiğine abone olabilmesi
    #   "engel_bilgi" ve "bilgi_servisi" servislerinin çalışıyor olması gerekmektedir. 
    #
    #______________________________________________________________________________________________

    def ana_fonksiyon(self):
        rospy.Subscriber ('/odom', Odometry, self.bas_aci_callback_fonksiyonu)
        rospy.Subscriber('/mesafe_hesabi', DistanceInfo, self.mesafe_hesabi_callback_fonksiyonu)
        self.robot_hiz_yayini = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.engel_bilgi_servisi = rospy.Service('engel_bilgi', ObstacleInfo, self.engel_bilgilendirme_servisi)
        self.bilgi_servisi = rospy.Service('bilgi_servisi', InfoService, self.gorev_bilgilendirme_servisi)
        
        # Saniyede 2 defa "while not rospy.is_shutdown()" döngüsü içindeki işlemleri gerçekleştirmektedir.
        rate = rospy.Rate(2)

        # Robotun hiz yayinini yapmak icin kullanilan mesaj tipidir.
        robot_hiz_mesaji = Twist()

        while not rospy.is_shutdown():
            if not self.robot_hareket_emir:
                # Engel var ise durmak için lineer ve açısal hızlar 0.0 olarak oluşturulur.
                self.robot_durma_fonksiyonu(robot_hiz_mesaji)

            else:
                # self.robot_hareket_emir değeri True'dur. Görevleri uygulamaya devam eder.
                self.robot_gorev_uygulama_fonksiyonu(robot_hiz_mesaji)

            rate.sleep()


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              gorev_okuma_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Görevler yaml dosyayısı okuyarak görevleri almaktadır.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   gecici_gorev_listesi                        list                    Okunmuş görevleri ifade eder.
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def gorev_okuma_fonksiyonu(self):
        okunan_gorevler_dosyasi = dict(rospy.get_param("~Missions"))
        gecici_gorev_listesi = list()

        for i in range(len(okunan_gorevler_dosyasi.keys())):
            anahtar = "Mission_" + str(i + 1)
            gecici_liste = [okunan_gorevler_dosyasi[str(anahtar)]["going_road"], okunan_gorevler_dosyasi[str(anahtar)]["head_angle"]]
            gecici_gorev_listesi.append(gecici_liste)

        return gecici_gorev_listesi


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              gorev_atama_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Okunmuş görevleri gorev_listesi değişkenine aktarır.
    #                               Görevlerin mesafe değerleri toplanarak toplam_gidilmesi_gereken_mesafe
    #                               değişkenine aktarılır. 
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def gorev_atama_fonksiyonu(self):
       # Görevin oluşturulması için kullanılan fonksiyondur.
       # Görevler [Gidilecek Mesafe, Baş Açı] formatındadır.
        self.gorev_listesi = self.gorev_okuma_fonksiyonu()

        for i in range(len(self.gorev_listesi)):
            # Görev boyunca gidilecek toplam mesafe hesaplanmaktadır.
            self.toplam_gidilmesi_gereken_mesafe += self.gorev_listesi[i][0]


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              robot_durma_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Robotun durması için çağırılan fonksiyondur. Lineer ve açısal
    #                               hız değerleri 0.0 olarak robot_hiz_yayini değişkeni üzerinden
    #                               yayınlamaktadır.  
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   robot_hiz_mesaji                            Twist                   Robotun hız verilerini içeren structure yapısında değişkendir. 
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def robot_durma_fonksiyonu(self, robot_hiz_mesaji):
        print("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\t\t\tEngel Var\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
        robot_hiz_mesaji.linear.x = 0.0
        robot_hiz_mesaji.angular.z = 0.0
        self.robot_hiz_yayini.publish(robot_hiz_mesaji)


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              robot_gorev_uygulama_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       gorev_listesi değişkeninde bulunan tüm görevleri gerçeklemektedir.
    #                               Robotun göreve göre lineer ve açısal hız değerlerini üretmekte ve
    #                               robot_hiz_yayini değişkeni üzerinden yayınlamaktadır.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   robot_hiz_mesaji                            Twist                   Robotun hız verilerini içeren structure yapısında değişkendir. 
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def robot_gorev_uygulama_fonksiyonu(self, robot_hiz_mesaji):
        # mevcut_gorev değeri gorev_listesi listesinin eleman sayısı ile karşılaştırır. Eğer mevcut_gorev değeri küçük ise
        # görevleri yapmaya devam etmektedir.
        if self.mevcut_gorev < len(self.gorev_listesi):
            # gorev_listesideki mevcut görevin 0. indisindeki değer görevde gidilmesi belirlinen mesafeyi
            # 1. indisindeki değer görevde yapması gereken başı açı değerini belirtmektedir.
            gorevde_gidilmesi_belirlenen_mesafe = self.gorev_listesi[self.mevcut_gorev][0]
            gorevde_donus_yapilacak_bas_aci = self.gorev_listesi[self.mevcut_gorev][1]

            # mevcut_gorevde_gidilmesi_gereken_mesafe değeri, mesafe_hesabi topiğinden okunan hesaplanmış mesafe bilgisi değerinden
            # tamamlanan görevler boyunca gidilmiş toplam mesafe arasındaki fark kadardır.
            # Bu değişken mevcut görevdeki gidilmesi gereken mesafe bulmak için kullanılmaktadır.
            mevcut_gorevde_gidilmesi_gereken_mesafe = self.mesafe_bilgisi_hesaplanan_mesafe - self.tamamlanan_gorevlerin_toplam_mesafesi

            # Mevcut görevde gidilmesi belirlenen mesafe değeri gidildikten sonra robot_donus_yap_emri True olur.
            # Bu değer True ise görevde belirlenen baş açı değerine göre açısal hız üretilerek uygulanmaktadır.
            if self.robot_donus_yap_emri:
                # Açısal hız değerini üretmek için robot_donus_yapma_fonksiyonunu çağırmaktadır.
                acisal_hiz = self.robot_donus_yapma_fonksiyonu(gorevde_donus_yapilacak_bas_aci)
                robot_hiz_mesaji.linear.x = 0.0
                robot_hiz_mesaji.angular.z = acisal_hiz

                self.robot_hiz_yayini.publish(robot_hiz_mesaji)

            else:
                # robot_donus_yap_emrinin değeri False ise robot lineer olarak hareket etmektedir.
                # Görevde gidilmesi belirlenen mesafe değeri ile mevcut görevde gidilmesi gereken mesafe değeri karşılaştırılır.
                # Eğer görevde gidilmesi gereken mesafe değeri küçük ise görevde gidilmesi belirlenen mesafe değerine ulaşana kadar
                # lineer hız değeri üreterek yayınlamaktadır.
                if mevcut_gorevde_gidilmesi_gereken_mesafe < gorevde_gidilmesi_belirlenen_mesafe:
                    robot_hiz_mesaji.linear.x = 0.2
                    robot_hiz_mesaji.angular.z = 0.0

                    self.robot_hiz_yayini.publish(robot_hiz_mesaji)

                # Eğer eşit yada büyük olur ise mevcut görevin tamamlandığı belli olur ve tamamlanan_gorevlerin_toplam_mesafesi değeri
                # güncellenmektedir. Robotun dönüş yapabilmesi için robot_donus_yap_emri değeri True olur.
                else:
                    self.tamamlanan_gorevlerin_toplam_mesafesi = self.mesafe_bilgisi_hesaplanan_mesafe
                    self.robot_donus_yap_emri = True

        # Görevler bitmiş ise Görev Bitti mesajı ekrana yazdırılır. Lineer ve açısal hız değerleri 0.0 olarak yayınlanarak 
        # robotun hareket işlemi sonlanmaktadır.
        else:
            print("@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
            print("\t\tGorev Bitti")
            print("@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")

            robot_hiz_mesaji.linear.x = 0.0
            robot_hiz_mesaji.angular.z = 0.0

            self.robot_hiz_yayini.publish(robot_hiz_mesaji)


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              robot_donus_yapma_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Robotun parametrede belirtilen baş açı değeri için uygulaması gereken
    #                               açısal hız değerini üretmek.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   hedef_bas_aci_derece                        float                   Robotun yapması gereken baş açı değerini derece
    #                                                                       olarak ifade etmektedir. 
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   acisal_hiz                                  float                   Robotun uygulaması gereken açısal hız değeri.
    #
    #   GEREKLILIK:
    #
    #
    #______________________________________________________________________________________________

    def robot_donus_yapma_fonksiyonu(self, hedef_bas_aci_derece):
        # Derece cinsinden olan hedef baş açı değerini radyan cinsine çevrilir.
        hedef_bas_aci_radyan = hedef_bas_aci_derece*math.pi/180
        # Mevcut baş açı ile hedef baş açı arasındaki fark alınır ve açısal referans hız değeri 
        # çarpılarak uygulanacak açısal hız değeri üretilir.
        acisal_hiz = self.acisal_referans_hizi * (hedef_bas_aci_radyan - self.robot_bas_aci)

        # Mevcut baş açı ile hedef baş açı arasındaki fark belirtilen baş açı tolerans değerinden küçük ise
        # robotun donus yapma fonksiyonu sonlanır, robot_donus_yap_emri değeri False olur
        # ve mevcut_gorev değeri 1 artarak diğer göreve geçmesi sağlanmaktadır.
        if abs(hedef_bas_aci_radyan - self.robot_bas_aci) < self.robot_bas_aci_tolerans_degeri:
            acisal_hiz = 0.0
            self.robot_donus_yap_emri = False
            print("Mevcut Gorev Bitti Digerine Gec")
            print("\n\n\n")
            self.mevcut_gorev += 1

        return acisal_hiz


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              bas_aci_callback_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Robotun robot_bas_aci değerini okuyabilmesi için abone olduğu "/odom"
    #                               topiğinin callback fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   odom_mesaji                                 Odometry                Topiğin değerinin okunması için gereken mesaj tipidir.  
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "/odom" topiğine abone olmak gerekmektedir. 
    #
    #______________________________________________________________________________________________

    def bas_aci_callback_fonksiyonu(self, odom_mesaji):
        # robot_bas_aci değerini bulmak için odom_mesaji.pose.pose.orientation değeri dinlenmektedir.
        roll = pitch = yaw = 0.0
        okunan_oryantasyon = odom_mesaji.pose.pose.orientation
        oryantasyon_listesi = [okunan_oryantasyon.x, okunan_oryantasyon.y, okunan_oryantasyon.z, okunan_oryantasyon.w]
        (roll, pitch, yaw) = euler_from_quaternion(oryantasyon_listesi)

        self.robot_bas_aci = yaw


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              engel_bilgilendirme_servisi
    #   FONKSIYON AÇIKLAMASI:       Robotun robot_hareket_emir değerini okuyabilmesi için istemci ile iletişim
    #                               kurduğu servis fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   istek                                       ObstacleInfo              İstemciden gelen hareket emri değerini okur.
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   yanit                                       ObstacleInfo              İstemciye gönderilen yanit değeridir.
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için istemcinin "engel_bilgi" servisi üzerinden istekte bulunması gerekmektedir.
    #
    #______________________________________________________________________________________________
 
    def engel_bilgilendirme_servisi(self, istek):
        # İstemciden gelen isteği robot_hareket_emir değişkenine aktarmaktadır.
        self.robot_hareket_emir = istek.istek
        # Yanıt olarak istemciye okunan değer tekrar gönderilir.
        yanit = self.robot_hareket_emir

        return ObstacleInfoResponse(yanit)


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              gorev_bilgilendirme_servisi
    #   FONKSIYON AÇIKLAMASI:       Robotun tamamlanan_gorev_yuzde_degeri değerini yanıt olarak gönderebilmesi
    #                               için istemci ile iletişim kurduğu servis fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   istek                                       InfoService            İstemciden gelen istek değerini okur.
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #   yanit                                       InfoService            İstemciye gönderilen tamamlanan_gorev_yuzde_degeri
    #                                                                       yanit değeridir.
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için istemcinin "bilgi_servisi" servisi üzerinden istekte bulunması gerekmektedir.
    #
    #______________________________________________________________________________________________

    def gorev_bilgilendirme_servisi(self, istek):
        # İstemciden gelen isteği okur.
        okunan_bilgi_servisi_istegi = istek.istek

        # Eğer gelen istek "Gorev Yuzdesi" değerine eşit ise yanit olarak tamamlanan_gorev_yuzde_degeri gönderilmektedir.
        if okunan_bilgi_servisi_istegi == "Gorev Yuzdesi":
            yanit = float(self.tamamlanan_gorev_yuzde_degeri)

        else:
            yanit = 0.0

        return InfoServiceResponse(yanit)


    #______________________________________________________________________________________________
    #
    #   FONKSIYON ADI:              mesafe_hesabi_callback_fonksiyonu
    #   FONKSIYON AÇIKLAMASI:       Robotun mesafe_bilgisi_hesaplanan_mesafe değerini okuyabilmesi için abone olduğu
    #                               "/mesafe_hesabi" topiğinin callback fonksiyonudur.
    #
    #
    #   PARAMETRELER:
    #       ADI										TIPI			        AÇIKLAMASI
    #   odom_mesaji                                 DistanceInfo           Topiğin değerinin okunması için gereken mesaj tipidir.  
    #
    #   DÖNÜS:
    #       ADI				                        TIPI				    AÇIKLAMASI
    #
    #   GEREKLILIK:
    #   Bu fonksiyon kullanılabilmesi için "/mesafe_hesabi" topiğine abone olmak gerekmektedir. 
    #
    #______________________________________________________________________________________________

    def mesafe_hesabi_callback_fonksiyonu(self, odom_mesaji):
        # mesafe_bilgisi_hesaplanan_mesafe değerini bulmak için odom_mesaji.mesafe_bilgisi değeri dinlenmektedir.
        self.mesafe_bilgisi_hesaplanan_mesafe = float(odom_mesaji.mesafe_bilgisi)
        self.tamamlanan_gorev_yuzde_degeri = float((self.mesafe_bilgisi_hesaplanan_mesafe / self.toplam_gidilmesi_gereken_mesafe) * 100)


if __name__ == '__main__':
    try:
        rospy.init_node('temizlik_robotu_supurme_dugumu', anonymous=True)

        # TemizlikRobotuSupurme() sınıfını çağırmaktadır.
        node = TemizlikRobotuSupurme()

    except rospy.ROSInterruptException:
        pass
