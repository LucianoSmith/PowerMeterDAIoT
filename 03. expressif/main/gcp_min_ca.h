/*
 * gcp_min_ca.h
 *
 *  Created on: May 29, 2020
 *      Author: leopoldo
 */

#ifndef GCP_MIN_CA_H_
#define GCP_MIN_CA_H_

/*
 * Certificado CA mínimo de Google Cloud Platform (IoT Core)
 * https://cloud.google.com/iot/docs/how-tos/mqtt-bridge#using_a_long-term_mqtt_domain
 * Cert primario: https://pki.goog/gtsltsr/gtsltsr.crt
 * Cert backup: https://pki.goog/gsr4/GSR4.crt
 * 
 * Convertir certificados a formato PEM:
 * openssl x509 -inform DER -in gtsltsr.crt -out primary.pem -text
 * openssl x509 -inform DER -in GSR4.crt -out secondary.pem -text
 * 
 * Copiar el fragmento desde la línea que contiene BEGIN, hasta la que contiene END.
 */
const char GCP_MIN_CA[] =

"-----BEGIN CERTIFICATE-----\n"
"-----END CERTIFICATE-----\n";

#endif /* GOOGLE_MIN_CA_H_ */
