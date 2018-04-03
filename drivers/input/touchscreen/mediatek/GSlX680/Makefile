

$(warning wiskywisky)
$(warning $(obj))
$(warning $(srctree))


# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GSlX680/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GSlX680/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  mtk_gslX680.o 
obj-y	+=	gsl_point_id.o
$(obj)/gsl_point_id.o: $(srctree)/$(obj)/gsl_point_id
	cp $(srctree)/$(obj)/gsl_point_id $(obj)/gsl_point_id.o
